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

            // 見本軌道のデータを保持
            std::vector<ssv> m_reference_trajectory;
            std::vector<ssv> m_estimates; // 推定結果を保持するメンバ変数
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
                std::cout << "Position3DTrackerのインスタンスが作成されました。" << std::endl;
            }

            // 初期分布のサンプリング
            // y1はfilterメソッドの第一引数
            //ssv q1Samp(const osv& y1) {
            //    ssv x1;
            //    // Initialize near the first observation
            //    for (int i = 0; i < 3; ++i) {
            //        x1(i) = m_reference_trajectory[0](i);
            //    }
            //    return x1;
            //}
            // 初期分布のサンプリング(状態はm_reference_trajectoryのインデックスを定義域とする。ver.)
            ssv q1Samp(const osv& y1) {
                ssv x1;
                for (int i = 0; i < 3; ++i) {
                    x1(i) = 0.0;
                }
                return x1;
            }

            // 初期分布の評価
            float_t logMuEv(const ssv& x1) {
                return 1.0; // Uniform prior
            }

            // 初期提案分布の評価（何のために使うかイマイチわからない）
            float_t logQ1Ev(const ssv& x1, const osv& y1) override {
                return 0.0; // 例として、一定の値を返す
            }

            // 純粋仮想関数の実装
            ssv propMu(const ssv& xtm1) override {
                // 状態遷移の提案分布
                return xtm1; // 例として、単純に前の状態を返す
            }

            //// 状態遷移のサンプリング
            //ssv fSamp(const ssv& xtm1) override {
            //    ssv xt;
            //    for (int i = 0; i < 3; ++i) {
            //        xt(i) = xtm1(i) + m_sys_noise * m_normal_dist(m_random_engine);
            //    }
            //    return xt;
            //}
            // 状態遷移のサンプリング（状態がインデックスver.）
            ssv fSamp(const ssv& xtm1) override {
                ssv xt;
                //float_t noise = m_sys_noise * m_normal_dist(m_random_engine);
                float_t noise = m_sys_noise * m_poisson_dist(m_random_engine);
                for (int i = 0; i < 3; ++i) {
                    xt(i) = xtm1(i) + static_cast<int>(noise) - 1;
                }
                // m_reference_trajectoryのインデックスを定義域とする
                if (xt(0) < 0) {
                    for (int i = 0; i < 3; ++i) {
                        xt(i) = 0;
                    }
                }
                else if (xt(0) > 719) {  // 719はm_reference_trajectoryの最大インデックス
                    for (int i = 0; i < 3; ++i) {
                        xt(i) = 719;
                    }
                }

                return xt;
            }

            // 観測尤度の評価
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



            // フィルタリング処理
            void filter(const osv& observation, const std::vector<std::function<const Eigen::MatrixXd(const ssv&)>>& eval_funcs) {
                // apfクラスのfilterメソッドを呼び出す
                APF<nparts, 3, 3, mn_resampler<nparts, float_t>, float_t>::filter(observation, eval_funcs);

                // 最新の推定結果を取得して保存
                ssv estimate = this->m_particles[9]; // 例として最初の粒子を使用
                m_estimates.push_back(estimate);

                // m_particles[0][0]～m_particles[9][0]までの先頭要素をm_estimates2にpush_backする
                ssv new_estimate;
                int index0 = this->m_particles[0](0);
                int index1 = this->m_particles[124](0);
                int index2 = this->m_particles[34](0);
                int index3 = this->m_particles[36](0);
                int index4 = this->m_particles[244](0);
                int index5 = this->m_particles[48](0);
                int index6 = this->m_particles[463](0);
                int index7 = this->m_particles[146](0);

                //this->m_particles[](0)の平均をindex9に格納
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



                // 時刻を更新
                updateTime();
            }

            // 推定結果を返す
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




            // 時刻を更新する
            void updateTime() {
                m_current_time++;
            }
        };
    }
}
#endif