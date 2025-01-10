#define UNITY_INTERFACE_API __stdcall
#define UNITY_INTERFACE_EXPORT __declspec(dllexport)

extern "C"
{
    UNITY_INTERFACE_EXPORT int UNITY_INTERFACE_API DoSomething(int x, int y)
    {
        const int x2 = x * 2;
        const int y3 = y * 3;
        return x2 * y3;
    }
}

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "auxiliary_pf.h"
#include "PositionTracker.h"
#include "pf_base.h"
#include "rv_samp.h"

namespace pf {

    namespace filters {

        // 見本軌道を生成する関数（例）
        std::vector<Eigen::Vector3d> generateReferenceTrajectory() {
            std::vector<Eigen::Vector3d> trajectory;
            // 例：円軌道
            for (int t = 0; t <= 400; ++t) {
                double angle = 2.0 * 3.14 * t / 400.0;
                Eigen::Vector3d pos;
                pos << std::cos(angle), std::sin(angle), t / 400.0;
                trajectory.push_back(pos);
            }
            return trajectory;
        }

        // CSVファイルから観測データを読み込む関数
        std::vector<Eigen::Vector3d> loadObservationsFromCSV(const std::string& file_path) {
            std::vector<Eigen::Vector3d> observations;
            std::ifstream file(file_path);
            std::string line;

            while (std::getline(file, line)) {
                std::stringstream ss(line);
                std::string value;
                Eigen::Vector3d obs;
                int i = 0;

                while (std::getline(ss, value, ',')) {
                    obs(i++) = std::stod(value);
                }

                observations.push_back(obs);
            }

            return observations;
        }

        // 推定結果をファイルに書き出す関数
        void saveEstimatesToFile(const std::vector<Eigen::Vector3d>& estimates, const std::string& file_path) {
            std::ofstream file(file_path);

            for (const auto& estimate : estimates) {
                file << estimate(0) << "," << estimate(1) << "," << estimate(2) << "\n";
            }
        }

        int main(int argc, char* argv[]) {
            if (argc < 3) {
                std::cerr << "Usage: " << argv[0] << " <observation_csv_path> <output_csv_path>" << std::endl;
                return 1;
            }
            std::string observation_csv_path = argv[1];
            std::string output_csv_path = argv[2];

            // 見本軌道の生成
            auto reference_trajectory = generateReferenceTrajectory();

            // パーティクル数を1000に設定
            constexpr size_t NUM_PARTICLES = 1000;

            // トラッカーの初期化
            Position3DTracker<NUM_PARTICLES, double> tracker(reference_trajectory, 0.1, 0.1);

            // 観測データの読み込み
            auto observations = loadObservationsFromCSV(observation_csv_path);

            // 評価したい関数（位置の推定値を取得）
            auto position_eval = [](const Eigen::Vector3d& state) -> const Eigen::MatrixXd {
                return state;
                };
            std::vector<std::function<const Eigen::MatrixXd(const Eigen::Vector3d&)>> eval_funcs;
            eval_funcs.push_back(position_eval);

            // 推定結果を保存するベクトル
            std::vector<Eigen::Vector3d> estimates;

            // フィルタリングの実行
            for (const auto& obs : observations) {
                tracker.filter(obs, eval_funcs);

                // 推定結果の取得
                auto estimate = tracker.getExpectations()[0];
                estimates.push_back(estimate);

                std::cout << "Estimated position: "
                    << estimate(0) << ", "
                    << estimate(1) << ", "
                    << estimate(2) << std::endl;

                // 時刻を更新
                tracker.updateTime();
            }

            // 推定結果をファイルに書き出す
            saveEstimatesToFile(estimates, output_csv_path);

            return 0;
        }
    }
}