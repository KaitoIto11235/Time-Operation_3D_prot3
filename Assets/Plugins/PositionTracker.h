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

            // 見本軌道のデータを保持
            std::vector<ssv> m_reference_trajectory;
            std::vector<ssv> m_estimates; // 推定結果を保持するメンバ変数
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

            // 初期分布の評価
            float_t logMuEv(const ssv& x1) {
                return 0.0; // Uniform prior
            }

            // 観測尤度の評価
            float_t logGEv(const osv& yt, const ssv& xt) {
                float_t sum = 0.0;
                // Gaussian observation model
                for (int i = 0; i < 3; ++i) {
                    float_t diff = yt(i) - xt(i);
                    sum += -0.5 * (diff * diff) / (m_obs_noise * m_obs_noise);
                }
                return sum;
            }

            // 次の状態をサンプリング
            ssv qSamp(const ssv& xtm1, const osv& yt) {
                ssv xt;

                // 見本軌道の次の位置を取得
                size_t next_time = m_current_time + 1;
                if (next_time < m_reference_trajectory.size()) {
                    // 見本軌道の次の位置にガウシアンノイズを加える
                    for (int i = 0; i < 3; ++i) {
                        xt(i) = m_reference_trajectory[next_time](i) +
                            m_sys_noise * m_normal_dist(m_random_engine);
                    }
                }
                else {
                    // 見本軌道の終端を超えた場合は現在位置の周りにノイズを加える
                    for (int i = 0; i < 3; ++i) {
                        xt(i) = xtm1(i) + m_sys_noise * m_normal_dist(m_random_engine);
                    }
                }
                return xt;
            }

            // 提案分布の評価
            float_t logQEv(const ssv& xt, const ssv& xtm1, const osv& yt) {
                float_t sum = 0.0;
                size_t next_time = m_current_time + 1;

                if (next_time < m_reference_trajectory.size()) {
                    // 見本軌道からの距離に基づく評価
                    for (int i = 0; i < 3; ++i) {
                        float_t diff = xt(i) - m_reference_trajectory[next_time](i);
                        sum += -0.5 * (diff * diff) / (m_sys_noise * m_sys_noise);
                    }
                }
                else {
                    // 見本軌道の終端を超えた場合
                    for (int i = 0; i < 3; ++i) {
                        float_t diff = xt(i) - xtm1(i);
                        sum += -0.5 * (diff * diff) / (m_sys_noise * m_sys_noise);
                    }
                }
                return sum;
            }

            // 状態遷移の評価
            float_t logFEv(const ssv& xt, const ssv& xtm1) {
                return logQEv(xt, xtm1, osv()); // 同じ評価を使用
            }

            // auxiliary weightの計算
            float_t logAuxWeightEv(const osv& yt, const ssv& xtm1) {
                ssv predicted = qSamp(xtm1, yt);  // 予測状態
                return logGEv(yt, predicted);      // 予測状態での観測尤度
            }

            // 純粋仮想関数の実装
            ssv propMu(const ssv& xtm1) override {
                // 状態遷移の提案分布
                return xtm1; // 例として、単純に前の状態を返す
            }

            ssv fSamp(const ssv& xtm1) override {
                // 状態遷移のサンプリング
                ssv xt;
                for (int i = 0; i < 3; ++i) {
                    xt(i) = xtm1(i) + m_sys_noise * m_normal_dist(m_random_engine);
                }
                return xt;
            }

            float_t logQ1Ev(const ssv& x1, const osv& y1) override {
                // 初期提案分布の評価
                return 0.0; // 例として、一定の値を返す
            }

            // フィルタリング処理
            void filter(const osv& observation, const std::vector<std::function<const Eigen::MatrixXd(const ssv&)>>& eval_funcs) {
                // シンプルなフィルタリング処理の例
                ssv estimate = qSamp(m_reference_trajectory[m_current_time], observation);
                m_estimates.push_back(estimate);
            }

            // 推定結果を返す
            std::vector<ssv> getExpectations() const {
                return m_estimates;
            }



            // 時刻を更新する
            void updateTime() {
                m_current_time++;
            }
        };
    }
}
#endif