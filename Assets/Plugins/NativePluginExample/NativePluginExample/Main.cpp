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

namespace pf {
    namespace filters {

        // CSV�t�@�C������ϑ��f�[�^��ǂݍ��ފ֐�
        std::vector<Eigen::Vector3d> loadFromCSV(const std::string& file_path) {
            std::vector<Eigen::Vector3d> data;
            std::ifstream file(file_path);
            std::string line;
            bool isFirstLine = true;

            while (std::getline(file, line)) {

                // �ŏ��̍s�̓w�b�_�[�Ȃ̂ŃX�L�b�v
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
                        // �G���[����������i���O�ɋL�^���đ��s�j
                        std::cerr << "CSV�ɖ����Ȑ��l������܂�: " << value << std::endl;
                        // ���s�ڂ���\��
                        std::cerr << "�s��: " << i << std::endl;
                        continue;
                    }
                }

                data.push_back(obs);
            }

            // data�̒��g����ł���΃G���[���o��
            if (data.empty()) {
                std::cerr << "�t�@�C������data�̓ǂݍ��݂Ɏ��s���܂����B" << file_path << std::endl;
            }
            return data;
        }

        // ���茋�ʂ��t�@�C���ɏ����o���֐�
        void saveEstimatesToFile(const std::vector<Eigen::Vector3d>& estimates, const std::string& file_path) {
            std::ofstream file(file_path);

            for (const auto& estimate : estimates) {
                file << estimate(0) << "," << estimate(1) << "," << estimate(2) << "\n";
            }
        }

        static void saveEstimatesToFile(const std::vector<Eigen::Vector3d>& estimates2,
            const std::vector<Eigen::Vector3d>& estimates3,
            const std::vector<Eigen::Vector3d>& estimates4,
            const std::string& file_path) {

            std::ofstream file(file_path);

            if (!file.is_open()) {
                std::cerr << "Failed to open file: " << file_path << std::endl;
                return;
            }

            if (!estimates2.empty()) {
                std::cout << "Successfully opened file: " << file_path << std::endl;
            }
            else
            {
                std::cerr << "No estimates to write to file: " << file_path << std::endl;
                return;
            }
            // estimates2, estimates3, estimates4�����ɘA�����ăt�@�C���ɏ�������
            for (int i = 0; i < estimates2.size(); i++) {
                file << estimates2[i](0) << "," << estimates2[i](1) << "," << estimates2[i](2) << ",";
                file << estimates3[i](0) << "," << estimates3[i](1) << "," << estimates3[i](2) << ",";
                file << estimates4[i](0) << "," << estimates4[i](1) << "," << estimates4[i](2) << "\n";
            }

            std::cout << "Successfully wrote estimates to file: " << file_path << std::endl;

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
        return 0;
    }
}

extern "C"
{
    UNITY_INTERFACE_EXPORT int UNITY_INTERFACE_API FinishTracker()
    {
        // std::unique_ptr�̃��������J�����鏈��
        tracker.reset();
		call_count = 0;
        return 0;
    }
}

extern "C"
{
    UNITY_INTERFACE_EXPORT int UNITY_INTERFACE_API ParticleFilter(double* userObserve, double* userTarget)
    {
        std::string reference_trajectory_csv_path = "Assets/Assets/File/AdaptModelMove.csv";
        std::string observation_csv_path = "Assets/Assets/File/AdaptUserTraining.csv";
        std::string output_csv_path = "Assets/Assets/File/output1.csv";

        //// ���{�O���̐���
        //auto reference_trajectory = pf::filters::loadFromCSV(reference_trajectory_csv_path);
        //// �ϑ��f�[�^�̓ǂݍ���
        //auto observations = pf::filters::loadFromCSV(observation_csv_path);

        //// �g���b�J�[�̏�����
        //tracker = std::make_unique<pf::filters::Position3DTracker<NUM_PARTICLES, double>>(reference_trajectory, 1.0, 1.0);

        // ���{�O���̐���
        auto reference_trajectory = pf::filters::loadFromCSV(reference_trajectory_csv_path);
        // �ϑ��f�[�^�̓ǂݍ���
        auto observations = pf::filters::loadFromCSV(observation_csv_path);

        // �g���b�J�[������������Ă��Ȃ��ꍇ�̂ݏ�����
        if (call_count == 0)
        {
            // �g���b�J�[�̏�����
            tracker = std::make_unique<pf::filters::Position3DTracker<NUM_PARTICLES, double>>(reference_trajectory, 1.0, 1.0);

        }

        if (call_count < 2100)
        {
            //userObserve��reference_trajectory�̒l����
            /*userObserve[0] = reference_trajectory[call_count](0);
            userObserve[1] = reference_trajectory[call_count](1);
            userObserve[2] = reference_trajectory[call_count](2);*/
            userObserve[0] = observations[call_count](0);
            userObserve[1] = observations[call_count](1);
            userObserve[2] = observations[call_count](2);
        }




        // �]���������֐��i�ʒu�̐���l���擾�j
        auto position_eval = [](const Eigen::Vector3d& state) -> const Eigen::MatrixXd {
            return state;
            };
        std::vector<std::function<const Eigen::MatrixXd(const Eigen::Vector3d&)>> eval_funcs;
        eval_funcs.push_back(position_eval);

        // ���茋�ʂ�ۑ�����x�N�g��
        std::vector<Eigen::Vector3d> estimates;

        //// �t�B���^�����O�̎��s
        //for (const auto& obs : observations) {
        //    tracker->filter(obs, eval_funcs);

        //    // �������X�V
        //    tracker->updateTime();
        //}



        // �t�B���^�����O�̎��s
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

        // ���茋�ʂ��t�@�C���ɏ����o��
        //pf::filters::saveEstimatesToFile(tracker.getExpectations(), output_csv_path);
        if (call_count == 8)
        {
            pf::filters::saveEstimatesToFile(tracker->getExpectations2(), tracker->getExpectations3(), tracker->getExpectations4(), output_csv_path);
        }

        return 0;
    }
}

