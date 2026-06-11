#include <rclcpp/rclcpp.hpp>
#include <vss_msgs/msg/ball_state.hpp>

#include <array>
#include <cmath>
#include <algorithm>

// ─────────────────────────────────────────────────────────────────────────────
//  EKFNode — Filtro de Kalman Estendido para rastreio da bola VSS
//
//  ESTADO:  x = [px, py, vx, vy]  (posição + velocidade em metros)
//
//  MODELO DE PROCESSO (predição a cada dt):
//    px += vx * dt
//    py += vy * dt
//    vx *= (1 - friction * dt)   ← atrito
//    vy *= (1 - friction * dt)
//
//  DETECÇÃO DE REBOTE:
//    Se a posição predita ultrapassa a parede do campo VSS:
//      vx ou vy inverte × coef_restituicao
//    Isso permite que o strategy_node já se posicione para o rebote
//    ANTES da bola fisicamente bater na parede.
//
//  MEDIÇÃO:  z = [px, py]  (SSL-Vision / FIRASim só dá posição)
//    vx e vy são estimados pelo filtro a partir de medições consecutivas
//
//  Subscriptions:  /ball_raw   (BallState — bruto do FIRASim)
//  Publications:   /ball_state (BallState — filtrado + velocidade estimada)
// ─────────────────────────────────────────────────────────────────────────────

// Matriz 4x4 como array flat [linha][coluna]
using Mat4 = std::array<std::array<double, 4>, 4>;
using Vec4 = std::array<double, 4>;

// Operações matriciais mínimas necessárias
static Mat4 mat4_add(const Mat4& A, const Mat4& B) {
    Mat4 C{};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            C[i][j] = A[i][j] + B[i][j];
    return C;
}

static Mat4 mat4_mul(const Mat4& A, const Mat4& B) {
    Mat4 C{};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                C[i][j] += A[i][k] * B[k][j];
    return C;
}

static Mat4 mat4_transpose(const Mat4& A) {
    Mat4 T{};
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            T[i][j] = A[j][i];
    return T;
}

// Inversa de matriz 2x2 (para S = H*P*Ht + R)
// Usamos só o bloco 2x2 superior esquerdo
static std::array<std::array<double,2>,2> inv2x2(
    double a, double b, double c, double d)
{
    double det = a*d - b*c;
    if (std::abs(det) < 1e-12) det = 1e-12;
    return {{ {{d/det, -b/det}}, {{-c/det, a/det}} }};
}

// ─────────────────────────────────────────────────────────────────────────────

class EKFNode : public rclcpp::Node {
public:
    EKFNode() : Node("ekf") {
        // ── Parâmetros ────────────────────────────────────────────────────
        declare_parameter("field_length",       1.50);
        declare_parameter("field_width",        1.30);
        declare_parameter("ball_friction",      0.05);   // atrito por segundo
        declare_parameter("wall_restitution",   0.80);   // coef. de restituição
        declare_parameter("process_noise_pos",  0.001);  // ruído Q — posição
        declare_parameter("process_noise_vel",  0.05);   // ruído Q — velocidade
        declare_parameter("measure_noise",      0.002);  // ruído R — medição
        declare_parameter("max_innovation",     0.30);   // rejeita saltos > 30cm

        field_half_x_   = get_parameter("field_length").as_double()     / 2.0;
        field_half_y_   = get_parameter("field_width").as_double()      / 2.0;
        friction_       = get_parameter("ball_friction").as_double();
        restitution_    = get_parameter("wall_restitution").as_double();
        max_innovation_ = get_parameter("max_innovation").as_double();

        double qp = get_parameter("process_noise_pos").as_double();
        double qv = get_parameter("process_noise_vel").as_double();
        double r  = get_parameter("measure_noise").as_double();

        // ── Matrizes de ruído ─────────────────────────────────────────────
        // Q = ruído do processo (incerteza do modelo)
        Q_ = {{
            {{qp, 0,  0,  0 }},
            {{0,  qp, 0,  0 }},
            {{0,  0,  qv, 0 }},
            {{0,  0,  0,  qv}}
        }};

        // R = ruído da medição (incerteza do sensor)
        // FIRASim é muito preciso → R pequeno
        R_[0][0] = r;  R_[0][1] = 0;
        R_[1][0] = 0;  R_[1][1] = r;

        // P = covariância inicial (alta incerteza)
        P_ = {{
            {{1, 0, 0, 0}},
            {{0, 1, 0, 0}},
            {{0, 0, 1, 0}},
            {{0, 0, 0, 1}}
        }};

        // H = matriz de observação [px, py, vx, vy] → [px, py]
        H_[0] = {1.0, 0.0};
        H_[1] = {0.0, 1.0};

        // ── ROS 2 ─────────────────────────────────────────────────────────
        pub_ball_ = create_publisher<vss_msgs::msg::BallState>(
            "/ball_state", rclcpp::SensorDataQoS());

        sub_ball_ = create_subscription<vss_msgs::msg::BallState>(
            "/ball_raw",
            rclcpp::SensorDataQoS(),
            std::bind(&EKFNode::onBallRaw, this, std::placeholders::_1));

        last_time_ = now();

        RCLCPP_INFO(get_logger(),
            "EKFNode iniciado — campo %.2fx%.2f m, atrito=%.3f",
            field_half_x_*2, field_half_y_*2, friction_);
    }

private:
    // ── Estado do EKF ─────────────────────────────────────────────────────
    Vec4 x_   = {0, 0, 0, 0};  // [px, py, vx, vy]
    Mat4 P_{};                  // covariância
    Mat4 Q_{};                  // ruído do processo
    std::array<std::array<double,2>,4> H_{};   // matriz de observação (2x4)
    std::array<std::array<double,2>,2> R_{};   // ruído da medição (2x2)

    double field_half_x_;
    double field_half_y_;
    double friction_;
    double restitution_;
    double max_innovation_;

    bool   initialized_ = false;
    rclcpp::Time last_time_;

    rclcpp::Publisher<vss_msgs::msg::BallState>::SharedPtr    pub_ball_;
    rclcpp::Subscription<vss_msgs::msg::BallState>::SharedPtr sub_ball_;

    // ─────────────────────────────────────────────────────────────────────
    //  Callback: nova medição da bola vinda do bridge
    // ─────────────────────────────────────────────────────────────────────
    void onBallRaw(const vss_msgs::msg::BallState::SharedPtr msg) {
        if (!msg->valid) return;

        auto now_time = now();
        double dt = (now_time - last_time_).seconds();
        last_time_ = now_time;

        // Clamp dt: evita saltos grandes se o nó ficou pausado
        dt = std::clamp(dt, 0.001, 0.1);

        // ── Inicialização na primeira medição ─────────────────────────────
        if (!initialized_) {
            x_[0] = msg->x;
            x_[1] = msg->y;
            // FIRASim já fornece velocidade — usamos como seed inicial
            x_[2] = msg->vx;
            x_[3] = msg->vy;
            initialized_ = true;
            publishState();
            return;
        }

        // ── 1. PREDIÇÃO ───────────────────────────────────────────────────
        predict(dt);

        // ── 2. INOVAÇÃO — verificar se medição é válida ───────────────────
        double innov_x = msg->x - x_[0];
        double innov_y = msg->y - x_[1];
        double innov_dist = std::hypot(innov_x, innov_y);

        // Rejeita medições com salto muito grande (outlier / artefato)
        if (innov_dist > max_innovation_) {
            // Reinicializa se o salto for enorme (>50cm = bola teletransportou)
            if (innov_dist > 0.5) {
                x_[0] = msg->x;  x_[1] = msg->y;
                x_[2] = msg->vx; x_[3] = msg->vy;
            }
            publishState();
            return;
        }

        // ── 3. ATUALIZAÇÃO (Update) ───────────────────────────────────────
        update(msg->x, msg->y);

        // ── 4. Publica estado filtrado ────────────────────────────────────
        publishState();
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Etapa de predição do EKF
    //  Propaga o estado x_ e a covariância P_ pelo modelo de processo
    // ─────────────────────────────────────────────────────────────────────
    void predict(double dt) {
        // ── Modelo de processo não-linear f(x) ───────────────────────────
        double friction_factor = 1.0 - friction_ * dt;
        friction_factor = std::max(0.0, friction_factor);

        double px_pred = x_[0] + x_[2] * dt;
        double py_pred = x_[1] + x_[3] * dt;
        double vx_pred = x_[2] * friction_factor;
        double vy_pred = x_[3] * friction_factor;

        // ── Detecção e tratamento de rebotes ─────────────────────────────
        // "Se a bola vai bater e voltar, o robô já deve se deslocar para
        //  o ponto de rebote antes mesmo do impacto" (requisito do projeto)
        if (px_pred > field_half_x_) {
            px_pred = 2.0 * field_half_x_ - px_pred;
            vx_pred = -vx_pred * restitution_;
        } else if (px_pred < -field_half_x_) {
            px_pred = -2.0 * field_half_x_ - px_pred;
            vx_pred = -vx_pred * restitution_;
        }

        if (py_pred > field_half_y_) {
            py_pred = 2.0 * field_half_y_ - py_pred;
            vy_pred = -vy_pred * restitution_;
        } else if (py_pred < -field_half_y_) {
            py_pred = -2.0 * field_half_y_ - py_pred;
            vy_pred = -vy_pred * restitution_;
        }

        x_[0] = px_pred;
        x_[1] = py_pred;
        x_[2] = vx_pred;
        x_[3] = vy_pred;

        // ── Jacobiano F = ∂f/∂x (linearização local) ─────────────────────
        // Para o modelo cinemático com atrito, F é quase linear:
        //   ∂px/∂px=1  ∂px/∂vx=dt
        //   ∂py/∂py=1  ∂py/∂vy=dt
        //   ∂vx/∂vx=friction_factor
        //   ∂vy/∂vy=friction_factor
        Mat4 F = {{
            {{1, 0, dt, 0 }},
            {{0, 1, 0,  dt}},
            {{0, 0, friction_factor, 0}},
            {{0, 0, 0, friction_factor}}
        }};

        // P = F * P * Ft + Q
        Mat4 Ft = mat4_transpose(F);
        P_ = mat4_add(mat4_mul(mat4_mul(F, P_), Ft), Q_);
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Etapa de atualização do EKF
    //  Incorpora a medição z = [px_meas, py_meas] ao estado
    // ─────────────────────────────────────────────────────────────────────
    void update(double px_meas, double py_meas) {
        // ── Inovação y = z - H*x ──────────────────────────────────────────
        double y0 = px_meas - x_[0];
        double y1 = py_meas - x_[1];

        // ── S = H * P * Ht + R  (covariância da inovação) ────────────────
        // Como H seleciona só px e py, S é 2x2:
        // S[0][0] = P[0][0] + R[0][0]
        // S[0][1] = P[0][1]
        // S[1][0] = P[1][0]
        // S[1][1] = P[1][1] + R[1][1]
        double s00 = P_[0][0] + R_[0][0];
        double s01 = P_[0][1] + R_[0][1];
        double s10 = P_[1][0] + R_[1][0];
        double s11 = P_[1][1] + R_[1][1];

        // ── S⁻¹ ───────────────────────────────────────────────────────────
        auto Si = inv2x2(s00, s01, s10, s11);

        // ── K = P * Ht * S⁻¹  (ganho de Kalman, 4x2) ────────────────────
        // PHt = P * Ht  → como H=[I2|0], PHt = P[0:2, :]t = colunas 0 e 1 de P
        double PHt[4][2];
        for (int i = 0; i < 4; i++) {
            PHt[i][0] = P_[i][0];
            PHt[i][1] = P_[i][1];
        }

        double K[4][2];
        for (int i = 0; i < 4; i++) {
            K[i][0] = PHt[i][0]*Si[0][0] + PHt[i][1]*Si[1][0];
            K[i][1] = PHt[i][0]*Si[0][1] + PHt[i][1]*Si[1][1];
        }

        // ── x = x + K * y ─────────────────────────────────────────────────
        for (int i = 0; i < 4; i++) {
            x_[i] += K[i][0]*y0 + K[i][1]*y1;
        }

        // ── P = (I - K*H) * P  (Joseph form para estabilidade numérica) ──
        // KH é 4x4: KH[i][j] = K[i][0]*H[0][j] + K[i][1]*H[1][j]
        // Como H seleciona px e py: KH[i][0]=K[i][0], KH[i][1]=K[i][1], resto 0
        Mat4 IKH = {{
            {{1-K[0][0], -K[0][1], 0, 0}},
            {{-K[1][0], 1-K[1][1], 0, 0}},
            {{-K[2][0], -K[2][1],  1, 0}},
            {{-K[3][0], -K[3][1],  0, 1}}
        }};
        P_ = mat4_mul(IKH, P_);
    }

    // ─────────────────────────────────────────────────────────────────────
    //  Publica o estado filtrado
    // ─────────────────────────────────────────────────────────────────────
    void publishState() {
        auto msg   = vss_msgs::msg::BallState();
        msg.x      = x_[0];
        msg.y      = x_[1];
        msg.vx     = x_[2];
        msg.vy     = x_[3];
        msg.valid  = true;
        pub_ball_->publish(msg);
    }
};

// ─────────────────────────────────────────────────────────────────────────────
//  main
// ─────────────────────────────────────────────────────────────────────────────
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EKFNode>());
    rclcpp::shutdown();
    return 0;
}
