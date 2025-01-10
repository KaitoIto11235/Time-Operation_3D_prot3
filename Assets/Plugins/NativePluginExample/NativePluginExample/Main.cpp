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

        // ���{�O���𐶐�����֐��i��j
        std::vector<Eigen::Vector3d> generateReferenceTrajectory() {
            std::vector<Eigen::Vector3d> trajectory;
            // ��F�~�O��
            for (int t = 0; t <= 400; ++t) {
                double angle = 2.0 * 3.14 * t / 400.0;
                Eigen::Vector3d pos;
                pos << std::cos(angle), std::sin(angle), t / 400.0;
                trajectory.push_back(pos);
            }
            return trajectory;
        }

        // CSV�t�@�C������ϑ��f�[�^��ǂݍ��ފ֐�
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

        // ���茋�ʂ��t�@�C���ɏ����o���֐�
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

            // ���{�O���̐���
            auto reference_trajectory = generateReferenceTrajectory();

            // �p�[�e�B�N������1000�ɐݒ�
            constexpr size_t NUM_PARTICLES = 1000;

            // �g���b�J�[�̏�����
            Position3DTracker<NUM_PARTICLES, double> tracker(reference_trajectory, 0.1, 0.1);

            // �ϑ��f�[�^�̓ǂݍ���
            auto observations = loadObservationsFromCSV(observation_csv_path);

            // �]���������֐��i�ʒu�̐���l���擾�j
            auto position_eval = [](const Eigen::Vector3d& state) -> const Eigen::MatrixXd {
                return state;
                };
            std::vector<std::function<const Eigen::MatrixXd(const Eigen::Vector3d&)>> eval_funcs;
            eval_funcs.push_back(position_eval);

            // ���茋�ʂ�ۑ�����x�N�g��
            std::vector<Eigen::Vector3d> estimates;

            // �t�B���^�����O�̎��s
            for (const auto& obs : observations) {
                tracker.filter(obs, eval_funcs);

                // ���茋�ʂ̎擾
                auto estimate = tracker.getExpectations()[0];
                estimates.push_back(estimate);

                std::cout << "Estimated position: "
                    << estimate(0) << ", "
                    << estimate(1) << ", "
                    << estimate(2) << std::endl;

                // �������X�V
                tracker.updateTime();
            }

            // ���茋�ʂ��t�@�C���ɏ����o��
            saveEstimatesToFile(estimates, output_csv_path);

            return 0;
        }
    }
}