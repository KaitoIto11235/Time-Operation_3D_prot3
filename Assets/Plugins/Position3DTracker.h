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
			std::uniform_int_distribution<int> m_uniform_dist;
            std::default_random_engine m_random_engine;

            // ���{�O���̃f�[�^��ێ�
            std::vector<ssv> m_reference_trajectory;
            std::vector<ssv> m_estimates; // ���茋�ʂ�ێ����郁���o�ϐ�
            /*std::vector<ssv> m_estimates2;
            std::vector<ssv> m_estimates3;
            std::vector<ssv> m_estimates4;*/
            ssv new_estimate2;
			ssv new_estimate3;
			ssv new_estimate4;

            // ���O�̊ϑ��l��ێ�����
			osv m_previous_observe;

            size_t m_current_time;

        public:
            // Constructor
            Position3DTracker(const std::vector<ssv>& reference_trajectory,
                float_t sys_noise = 0.1,
                float_t obs_noise = 0.1,
                float_t mean_normal = 1.0)
                : APF<nparts, 3, 3, mn_resampler<nparts, float_t>, float_t>()
                , m_sys_noise(sys_noise)
                , m_obs_noise(obs_noise)
                , m_normal_dist(mean_normal, 1)
                , m_poisson_dist(1)
                , m_uniform_dist(0, 1)
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
                ssv xt;
                /*int progress = m_uniform_dist(m_random_engine);
                float_t noise = m_normal_dist(m_random_engine);*/
                // ��ԑJ�ڂ̒�ĕ��z
                for (int i = 0; i < 3; ++i) {
                    xt(i) = xtm1(i);
                }
                return xt;
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

				int progress = m_uniform_dist(m_random_engine);
                float_t noise = m_normal_dist(m_random_engine);
                int jumpNoise = m_poisson_dist(m_random_engine);

                // �߂����ɋN����Ȃ����Ƃ��N�����ꍇ�A�W�����v������B
                if (jumpNoise > 4)
                {
                    jumpNoise = 1;
                }
                else
                {
                    jumpNoise = 0;
                }

                for (int i = 0; i < 3; ++i) {
                    xt(i) = xtm1(i) + std::round(noise) + (8 * jumpNoise);
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
				// ���xt��m_reference_trajectory�̃C���f�b�N�X�Ƃ��Ĉ���
                int index = static_cast<int>(xt(0));
                if (index < 0) {
                    index = 0;
				}
				else if (index > 719) {
					index = 719;
				}
                // Gaussian observation model
                if (index != 0)
                {
                    float_t diff_pos[3];
                    float_t diff_vel[3];
                    float_t diff;
                    for (int i = 0; i < 3; ++i) {

                        // �ʒu�̍���
                        float_t vel = 50.0 * yt(i) - 50.0 * m_previous_observe[i];
                        float_t ref_vel = 50.0 * m_reference_trajectory[index](i) - 50.0 * m_reference_trajectory[index - 1](i);

                        // �ʒu�̃Y��
                        diff_pos[i] = yt(i) - m_reference_trajectory[index](i);
                        // ���x�̃Y��
                        diff_vel[i] = vel - ref_vel;

                        //// �ʒu�̃Y���Ƒ��x�̃Y���̍��v
                        //diff = 0.9 * std::abs(diff_pos) + 0.1 * std::abs(diff_vel);
                        //sum += -0.5 * diff / (m_obs_noise * m_obs_noise);
                        //sum += std::log(2 * diff);
                    }
                    diff = 0.9 * std::sqrt(diff_pos[0] * diff_pos[0] + diff_pos[1] * diff_pos[1] + diff_pos[2] * diff_pos[2])
                        + 0.1 * std::sqrt(diff_vel[0] * diff_vel[0] + diff_vel[1] * diff_vel[1] + diff_vel[2] * diff_vel[2]);
                    sum = std::log(1 * diff);
				}
				else
				{
					for (int i = 0; i < 3; ++i) {
						float_t diff = yt(i) - m_reference_trajectory[index](i);
						sum += -0.5 * (diff * diff) / (m_obs_noise * m_obs_noise);
					}
				}

                // sum������������ꍇ�̑Ώ�
                if (sum < std::numeric_limits<float_t>::epsilon()) {
                    sum = -std::numeric_limits<float_t>::epsilon();
                }
                
                return sum;

                //// Gaussian observation model
                //for (int i = 0; i < 3; ++i) {
                //    float_t diff = yt(i) - m_reference_trajectory[index](i);
                //    sum += -0.5 * (diff * diff) / (m_obs_noise * m_obs_noise);
                //}
                //return sum;
            }



            // �t�B���^�����O����
            void filter(const osv& observation, const std::vector<std::function<const Eigen::MatrixXd(const ssv&)>>& eval_funcs) {
                // apf�N���X��filter���\�b�h���Ăяo��
                APF<nparts, 3, 3, mn_resampler<nparts, float_t>, float_t>::filter(observation, eval_funcs);

                // �ŐV�̐��茋�ʂ��擾���ĕۑ�
                //ssv estimate = this->m_particles[9]; // ��Ƃ��čŏ��̗��q���g�p
                //m_estimates.push_back(estimate);

                // m_particles[0][0]�`m_particles[9][0]�܂ł̐擪�v�f��m_estimates2��push_back����
                
                int index0 = this->m_particles[0](0);
                int index1 = this->m_particles[24](0);
                int index2 = this->m_particles[34](0);
                int index3 = this->m_particles[36](0);
                int index4 = this->m_particles[44](0);
                int index5 = this->m_particles[48](0);
                int index6 = this->m_particles[63](0);
                int index7 = this->m_particles[16](0);
                int index8 = this->m_particles[6](0);
                ////this->m_particles[](0)�̕��ς�index9�Ɋi�[
                //int sum = 0;
                //for (int i = 0; i < 1000; i++) {
                //    sum += this->m_particles[i](0);
                //}
                //int index8 = sum / 1000;

				// m_particles�̏d�ݕt�����ς����߂�
				float_t sum = 0.0;
				float_t sum_x = 0.0;
				for (int i = 0; i < nparts; i++) {
					sum += std::exp(this->m_logUnNormWeights[i]);
					sum_x += std::exp(this->m_logUnNormWeights[i]) * this->m_particles[i](0);
				}
				int index9 = sum_x / sum;

                new_estimate2 << this->m_reference_trajectory[index0](0), this->m_reference_trajectory[index1](0), this->m_reference_trajectory[index2](0);
                //m_estimates2.push_back(new_estimate);
                
                new_estimate3 << this->m_reference_trajectory[index3](0), this->m_reference_trajectory[index4](0), this->m_reference_trajectory[index5](0);
                //m_estimates3.push_back(new_estimate);

                new_estimate4 << this->m_reference_trajectory[index6](0), this->m_reference_trajectory[index7](0), this->m_reference_trajectory[index9](0);
                //m_estimates4.push_back(new_estimate);

                ssv estimate = this->m_reference_trajectory[index9];
                m_estimates.push_back(estimate);

				// ���O�̏�Ԃ�ێ�
                m_previous_observe = observation;

                // �������X�V
                updateTime();
            }

            // ���茋�ʂ�Ԃ�
            std::vector<ssv> getExpectations() const {
                return m_estimates;
            }
            
            // �������̐��茋�ʂ�Ԃ�
            const ssv getExpectations2() const {
                return new_estimate2;
            }
            const ssv getExpectations3() const {
                return new_estimate3;
            }
            const ssv getExpectations4() const {
                return new_estimate4;
            }





            // �������X�V����
            void updateTime() {
                m_current_time++;
            }
        };
    }
}
#endif