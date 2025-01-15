#pragma once
#ifndef POSITION_3DTRACKER_H
#define POSITION_3DTRACKER_H

#pragma once
#include <Eigen/Dense>
#include "auxiliary_pf.h"
#include "mn_resampler.h"
#include <vector>
#include <random>
#include <cmath>

namespace pf {

    namespace filters {


        // 3D position tracker using auxiliary particle filter
        //class Position3DTracker : public APF<nparts, 3, 3, mn_resampler<nparts, 3, float_t>, float_t>
        template<size_t nparts, typename float_t>
        class Position3DTracker : public APF<nparts, 3, 3, mn_resampler<nparts, float_t>, float_t>
        {
        public:
            using ssv = Eigen::Matrix<float_t, 3, 1>;  // State vector (x,y,z)
            using osv = Eigen::Matrix<float_t, 3, 1>;  // Observation vector (x,y,z)

        private:
            float_t m_sys_noise;    // System noise standard deviation
            float_t m_obs_noise;    // Observation noise standard deviation
            std::normal_distribution<float_t> m_normal_dist;
            std::poisson_distribution<int> m_poisson_dist;
            std::default_random_engine m_random_engine;

            // ���{�O���̃f�[�^��ێ�
            std::vector<ssv> m_reference_trajectory;
            std::vector<ssv> m_estimates; // ���茋�ʂ�ێ����郁���o�ϐ�
            std::vector<ssv> m_estimates2;
            std::vector<ssv> m_estimates3;
            std::vector<ssv> m_estimates4;
            size_t m_current_time;

        public:
            // Constructor
            Position3DTracker(const std::vector<ssv>& reference_trajectory,
                float_t sys_noise = 0.1,
                float_t obs_noise = 0.1)
                : APF<nparts, 3, 3, mn_resampler<nparts, float_t>, float_t>()
                , m_sys_noise(sys_noise)
                , m_obs_noise(obs_noise)
                , m_normal_dist(0.0, 1.0)
                , m_poisson_dist(2)
                , m_reference_trajectory(reference_trajectory)
                , m_current_time(0)
            {
                std::cout << "Position3DTracker�̃C���X�^���X���쐬����܂����B" << std::endl;
            }

            // �������z�̃T���v�����O
            // y1��filter���\�b�h�̑�����
            //ssv q1Samp(const osv& y1) {
            //    ssv x1;
            //    // Initialize near the first observation
            //    for (int i = 0; i < 3; ++i) {
            //        x1(i) = m_reference_trajectory[0](i);
            //    }
            //    return x1;
            //}
            // �������z�̃T���v�����O(��Ԃ�m_reference_trajectory�̃C���f�b�N�X���`��Ƃ���Bver.)
            ssv q1Samp(const osv& y1) {
                ssv x1;
                for (int i = 0; i < 3; ++i) {
                    x1(i) = 0.0;
                }
                return x1;
            }

            // �������z�̕]��
            float_t logMuEv(const ssv& x1) {
                return 1.0; // Uniform prior
            }

            // ������ĕ��z�̕]���i���̂��߂Ɏg�����C�}�C�`�킩��Ȃ��j
            float_t logQ1Ev(const ssv& x1, const osv& y1) override {
                return 0.0; // ��Ƃ��āA���̒l��Ԃ�
            }

            // �������z�֐��̎���
            ssv propMu(const ssv& xtm1) override {
                // ��ԑJ�ڂ̒�ĕ��z
                return xtm1; // ��Ƃ��āA�P���ɑO�̏�Ԃ�Ԃ�
            }

            //// ��ԑJ�ڂ̃T���v�����O
            //ssv fSamp(const ssv& xtm1) override {
            //    ssv xt;
            //    for (int i = 0; i < 3; ++i) {
            //        xt(i) = xtm1(i) + m_sys_noise * m_normal_dist(m_random_engine);
            //    }
            //    return xt;
            //}
            // ��ԑJ�ڂ̃T���v�����O�i��Ԃ��C���f�b�N�Xver.�j
            ssv fSamp(const ssv& xtm1) override {
                ssv xt;
                //float_t noise = m_sys_noise * m_normal_dist(m_random_engine);
                float_t noise = m_sys_noise * m_poisson_dist(m_random_engine);
                for (int i = 0; i < 3; ++i) {
                    xt(i) = xtm1(i) + static_cast<int>(noise) - 1;
                }
                // m_reference_trajectory�̃C���f�b�N�X���`��Ƃ���
                if (xt(0) < 0) {
                    for (int i = 0; i < 3; ++i) {
                        xt(i) = 0;
                    }
                }
                else if (xt(0) > 719) {  // 719��m_reference_trajectory�̍ő�C���f�b�N�X
                    for (int i = 0; i < 3; ++i) {
                        xt(i) = 719;
                    }
                }

                return xt;
            }

            // �ϑ��ޓx�̕]��
            float_t logGEv(const osv& yt, const ssv& xt) {
                float_t sum = 0.0;
                int index = static_cast<int>(xt(0));
                if (index < 0) {
                    index = 0;
                }
                // Gaussian observation model
                for (int i = 0; i < 3; ++i) {
                    float_t diff = yt(i) - m_reference_trajectory[index](i);
                    sum += -0.5 * (diff * diff) / (m_obs_noise * m_obs_noise);
                }
                return sum;
            }



            // �t�B���^�����O����
            void filter(const osv& observation, const std::vector<std::function<const Eigen::MatrixXd(const ssv&)>>& eval_funcs) {
                // apf�N���X��filter���\�b�h���Ăяo��
                APF<nparts, 3, 3, mn_resampler<nparts, float_t>, float_t>::filter(observation, eval_funcs);

                // �ŐV�̐��茋�ʂ��擾���ĕۑ�
                ssv estimate = this->m_particles[9]; // ��Ƃ��čŏ��̗��q���g�p
                m_estimates.push_back(estimate);

                // m_particles[0][0]�`m_particles[9][0]�܂ł̐擪�v�f��m_estimates2��push_back����
                ssv new_estimate;
                int index0 = this->m_particles[0](0);
                int index1 = this->m_particles[124](0);
                int index2 = this->m_particles[34](0);
                int index3 = this->m_particles[36](0);
                int index4 = this->m_particles[244](0);
                int index5 = this->m_particles[48](0);
                int index6 = this->m_particles[463](0);
                int index7 = this->m_particles[146](0);

                //this->m_particles[](0)�̕��ς�index9�Ɋi�[
                int sum = 0;
                for (int i = 0; i < 1000; i++) {
                    sum += this->m_particles[i](0);
                }
                int index8 = sum / 1000;


                new_estimate << this->m_reference_trajectory[index0](0), this->m_reference_trajectory[index1](0), this->m_reference_trajectory[index2](0);
                m_estimates2.push_back(new_estimate);
                new_estimate << this->m_reference_trajectory[index3](0), this->m_reference_trajectory[index4](0), this->m_reference_trajectory[index5](0);
                m_estimates3.push_back(new_estimate);
                new_estimate << this->m_reference_trajectory[index6](0), this->m_reference_trajectory[index7](0), this->m_reference_trajectory[index8](0);
                m_estimates4.push_back(new_estimate);



                // �������X�V
                updateTime();
            }

            // ���茋�ʂ�Ԃ�
            std::vector<ssv> getExpectations() const {
                return m_estimates;
            }
            std::vector<ssv> getExpectations2() const {
                return m_estimates2;
            }
            std::vector<ssv> getExpectations3() const {
                return m_estimates3;
            }
            std::vector<ssv> getExpectations4() const {
                return m_estimates4;
            }




            // �������X�V����
            void updateTime() {
                m_current_time++;
            }
        };
    }
}
#endif