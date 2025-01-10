#ifndef POSITION_TRACKER_H
#define POSITION_TRACKER_H

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
            std::default_random_engine m_random_engine;

            // ���{�O���̃f�[�^��ێ�
            std::vector<ssv> m_reference_trajectory;
            std::vector<ssv> m_estimates; // ���茋�ʂ�ێ����郁���o�ϐ�
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
                , m_reference_trajectory(reference_trajectory)
                , m_current_time(0)
            {
            }

            // Initial state distribution
            ssv q1Samp(const osv& y1) {
                ssv x1;
                // Initialize near the first observation
                for (int i = 0; i < 3; ++i) {
                    x1(i) = y1(i) + m_sys_noise * m_normal_dist(m_random_engine);
                }
                return x1;
            }

            // �������z�̕]��
            float_t logMuEv(const ssv& x1) {
                return 0.0; // Uniform prior
            }

            // �ϑ��ޓx�̕]��
            float_t logGEv(const osv& yt, const ssv& xt) {
                float_t sum = 0.0;
                // Gaussian observation model
                for (int i = 0; i < 3; ++i) {
                    float_t diff = yt(i) - xt(i);
                    sum += -0.5 * (diff * diff) / (m_obs_noise * m_obs_noise);
                }
                return sum;
            }

            // ���̏�Ԃ��T���v�����O
            ssv qSamp(const ssv& xtm1, const osv& yt) {
                ssv xt;

                // ���{�O���̎��̈ʒu���擾
                size_t next_time = m_current_time + 1;
                if (next_time < m_reference_trajectory.size()) {
                    // ���{�O���̎��̈ʒu�ɃK�E�V�A���m�C�Y��������
                    for (int i = 0; i < 3; ++i) {
                        xt(i) = m_reference_trajectory[next_time](i) +
                            m_sys_noise * m_normal_dist(m_random_engine);
                    }
                }
                else {
                    // ���{�O���̏I�[�𒴂����ꍇ�͌��݈ʒu�̎���Ƀm�C�Y��������
                    for (int i = 0; i < 3; ++i) {
                        xt(i) = xtm1(i) + m_sys_noise * m_normal_dist(m_random_engine);
                    }
                }
                return xt;
            }

            // ��ĕ��z�̕]��
            float_t logQEv(const ssv& xt, const ssv& xtm1, const osv& yt) {
                float_t sum = 0.0;
                size_t next_time = m_current_time + 1;

                if (next_time < m_reference_trajectory.size()) {
                    // ���{�O������̋����Ɋ�Â��]��
                    for (int i = 0; i < 3; ++i) {
                        float_t diff = xt(i) - m_reference_trajectory[next_time](i);
                        sum += -0.5 * (diff * diff) / (m_sys_noise * m_sys_noise);
                    }
                }
                else {
                    // ���{�O���̏I�[�𒴂����ꍇ
                    for (int i = 0; i < 3; ++i) {
                        float_t diff = xt(i) - xtm1(i);
                        sum += -0.5 * (diff * diff) / (m_sys_noise * m_sys_noise);
                    }
                }
                return sum;
            }

            // ��ԑJ�ڂ̕]��
            float_t logFEv(const ssv& xt, const ssv& xtm1) {
                return logQEv(xt, xtm1, osv()); // �����]�����g�p
            }

            // auxiliary weight�̌v�Z
            float_t logAuxWeightEv(const osv& yt, const ssv& xtm1) {
                ssv predicted = qSamp(xtm1, yt);  // �\�����
                return logGEv(yt, predicted);      // �\����Ԃł̊ϑ��ޓx
            }

            // �������z�֐��̎���
            ssv propMu(const ssv& xtm1) override {
                // ��ԑJ�ڂ̒�ĕ��z
                return xtm1; // ��Ƃ��āA�P���ɑO�̏�Ԃ�Ԃ�
            }

            ssv fSamp(const ssv& xtm1) override {
                // ��ԑJ�ڂ̃T���v�����O
                ssv xt;
                for (int i = 0; i < 3; ++i) {
                    xt(i) = xtm1(i) + m_sys_noise * m_normal_dist(m_random_engine);
                }
                return xt;
            }

            float_t logQ1Ev(const ssv& x1, const osv& y1) override {
                // ������ĕ��z�̕]��
                return 0.0; // ��Ƃ��āA���̒l��Ԃ�
            }

            // �t�B���^�����O����
            void filter(const osv& observation, const std::vector<std::function<const Eigen::MatrixXd(const ssv&)>>& eval_funcs) {
                // �V���v���ȃt�B���^�����O�����̗�
                ssv estimate = qSamp(m_reference_trajectory[m_current_time], observation);
                m_estimates.push_back(estimate);
            }

            // ���茋�ʂ�Ԃ�
            std::vector<ssv> getExpectations() const {
                return m_estimates;
            }



            // �������X�V����
            void updateTime() {
                m_current_time++;
            }
        };
    }
}
#endif