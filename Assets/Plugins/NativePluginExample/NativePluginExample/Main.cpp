#define UNITY_INTERFACE_API __stdcall
#define UNITY_INTERFACE_EXPORT __declspec(dllexport)

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Dense>
#include "auxiliary_pf.h"
#include "Position3DTracker.h"
#include "pf_base.h"
#include "rv_samp.h"

constexpr size_t NUM_PARTICLES = 1000;
std::unique_ptr<pf::filters::Position3DTracker<NUM_PARTICLES, double>> tracker;
int call_count = 0;
// 書き込みファイルの宣言
std::ofstream file;

namespace pf {
    namespace filters {

        // CSVファイルから観測データを読み込む関数
        std::vector<Eigen::Vector3d> loadFromCSV(const std::string& file_path) {
            std::vector<Eigen::Vector3d> data;
            std::ifstream file(file_path);
            std::string line;
            bool isFirstLine = true;

            while (std::getline(file, line)) {

                // 最初の行はヘッダーなのでスキップ
                if (isFirstLine) {
                    isFirstLine = false;
                    continue;
                }

                std::stringstream ss(line);
                std::string value;
                Eigen::Vector3d obs;
                int i = 0;

                while (std::getline(ss, value, ',')) {
                    try {
                        obs(i++) = std::stod(value);
                    }
                    catch (const std::invalid_argument& e) {
                        // エラーを処理する（ログに記録して続行）
                        std::cerr << "CSVに無効な数値があります: " << value << std::endl;
                        // 何行目かを表示
                        std::cerr << "行数: " << i << std::endl;
                        continue;
                    }
                }

                data.push_back(obs);
            }

            // dataの中身が空であればエラーを出力
            if (data.empty()) {
                std::cerr << "ファイルからdataの読み込みに失敗しました。" << file_path << std::endl;
            }
            return data;
        }

        // 推定結果をファイルに書き出す関数
        void saveEstimatesToFile(const std::vector<Eigen::Vector3d>& estimates, const std::string& file_path) {
            std::ofstream file(file_path);

            for (const auto& estimate : estimates) {
                file << estimate(0) << "," << estimate(1) << "," << estimate(2) << "\n";
            }
        }

        static void saveEstimatesToFile
            (const Eigen::Matrix<double, 3, 1>& estimates2,
            const Eigen::Matrix<double, 3, 1>& estimates3,
            const Eigen::Matrix<double, 3, 1>& estimates4,
            std::ofstream& file) 
        {

            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << std::endl;
                return;
            }

            // estimates2, estimates3, estimates4を横に連結してファイルに書き込む
            /*for (int i = 0; i < estimates2.size(); i++) {
                file << estimates2[i](0) << "," << estimates2[i](1) << "," << estimates2[i](2) << ",";
                file << estimates3[i](0) << "," << estimates3[i](1) << "," << estimates3[i](2) << ",";
                file << estimates4[i](0) << "," << estimates4[i](1) << "," << estimates4[i](2) << "\n";
            }*/

			// ファイルに書き込む
			file << estimates2(0) << "," << estimates2(1) << "," << estimates2(2) << ",";
			file << estimates3(0) << "," << estimates3(1) << "," << estimates3(2) << ",";
			file << estimates4(0) << "," << estimates4(1) << "," << estimates4(2) << "\n";

        }
    }
}


extern "C"
{
    UNITY_INTERFACE_EXPORT int UNITY_INTERFACE_API DoSomething(int x, int y)
    {
        const int x2 = x * 2;
        const int y3 = y * 3;
        return x2 * y3;
    }
}

extern "C"
{
    UNITY_INTERFACE_EXPORT int UNITY_INTERFACE_API ResetCount()
    {
		call_count = 0;
        // ファイルに空の行を追加
		if (file.is_open()) {
			file << "\n";
		}

        return 0;
    }
}

extern "C"
{
    UNITY_INTERFACE_EXPORT int UNITY_INTERFACE_API FinishTracker()
    {
        // std::unique_ptrのメモリを開放する処理
        tracker.reset();
		call_count = 0;
        // ファイルを閉じる
        if (file.is_open()) {
            file.close();
        }
        return 0;
    }
}

extern "C"
{
    UNITY_INTERFACE_EXPORT int UNITY_INTERFACE_API ParticleFilter(double* model, double* userObserve, double* userTarget)
    {
        std::string reference_trajectory_csv_path = "Assets/MoveFile/AdaptModelMove10mean.csv";
        std::string observation_csv_path = "Assets/MoveFile/AdaptUserMove10mean.csv";
        std::string output_csv_path = "Assets/MoveFile/output1.csv";

        //// 見本軌道の生成
        //auto reference_trajectory = pf::filters::loadFromCSV(reference_trajectory_csv_path);
        //// 観測データの読み込み
        //auto observations = pf::filters::loadFromCSV(observation_csv_path);

        //// トラッカーの初期化
        //tracker = std::make_unique<pf::filters::Position3DTracker<NUM_PARTICLES, double>>(reference_trajectory, 1.0, 1.0);

        // 見本軌道の生成
        auto reference_trajectory = pf::filters::loadFromCSV(reference_trajectory_csv_path);
        // 観測データの読み込み
        auto observations = pf::filters::loadFromCSV(observation_csv_path);

        // 書き込み先のファイルが開かれていなければ開く
        if (!file.is_open())
        {
            file.open(output_csv_path, std::ios::out | std::ios::app);

            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << output_csv_path << std::endl;
                return -1;
            }
        }

        // トラッカーが初期化されていない場合のみ初期化
        if (call_count == 0)
        {
            // トラッカーの初期化
            tracker = std::make_unique<pf::filters::Position3DTracker<NUM_PARTICLES, double>>
                (reference_trajectory, 1.0, 0.1, 720.0/1138.0);

        }

        if (call_count < 1147 && call_count > -1)
        {
            if (call_count < 719)
            {
                //modelにreference_trajectoryの値を代入(本当はC#側で十分できるけど、面倒くさい。)
                model[0] = reference_trajectory[call_count](0);
                model[1] = reference_trajectory[call_count](1);
                model[2] = reference_trajectory[call_count](2);
            }
			

            //userObserveにreference_trajectoryの値を代入
            userObserve[0] = observations[call_count](0);
            userObserve[1] = observations[call_count](1);
            userObserve[2] = observations[call_count](2);
        }




        // 評価したい関数（位置の推定値を取得）
        auto position_eval = [](const Eigen::Vector3d& state) -> const Eigen::MatrixXd {
            return state;
            };
        std::vector<std::function<const Eigen::MatrixXd(const Eigen::Vector3d&)>> eval_funcs;
        eval_funcs.push_back(position_eval);

        // フィルタリングの実行
        Eigen::Vector3d observation(userObserve[0], userObserve[1], userObserve[2]);
        tracker->filter(observation, eval_funcs);
        tracker->updateTime();

		if (call_count >= 1 && call_count <= 2100)
		{
            userTarget[0] = tracker->getExpectations()[call_count - 1](0);
            userTarget[1] = tracker->getExpectations()[call_count - 1](1);
            userTarget[2] = tracker->getExpectations()[call_count - 1](2);
		}
        call_count++;

        // 現在の推定結果をファイルに書き出す
        pf::filters::saveEstimatesToFile(tracker->getExpectations2(), tracker->getExpectations3(), tracker->getExpectations4(), file);
        

        return 0;
    }
}

