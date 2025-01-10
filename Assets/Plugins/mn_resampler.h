// mn_resampler.h
#ifndef MN_RESAMPLER_H
#define MN_RESAMPLER_H

#include "rv_samp.h"
#include <array>
#include <numeric>

namespace pf {
    namespace filters {

        template<size_t nparts, typename float_t>
        class mn_resampler
        {
        public:
            using array_float_t = std::array<float_t, nparts>;
            using array_uint_t = std::array<unsigned int, nparts>;

            mn_resampler() = default;

            array_uint_t resample(const array_float_t& log_weights) {
                // ���O�d�݂𐳋K�����Ďw���֐���K�p
                float_t max_log_weight = *std::max_element(log_weights.begin(), log_weights.end());
                array_float_t weights;
                std::transform(log_weights.begin(), log_weights.end(), weights.begin(),
                    [max_log_weight](float_t lw) { return std::exp(lw - max_log_weight); });

                // �d�݂̍��v���v�Z
                float_t sum_weights = std::accumulate(weights.begin(), weights.end(), float_t(0));

                // �d�݂𐳋K��
                std::transform(weights.begin(), weights.end(), weights.begin(),
                    [sum_weights](float_t w) { return w / sum_weights; });

                // ���U���z���g�p���ăT���v�����O
                std::discrete_distribution<> dist(weights.begin(), weights.end());
                std::mt19937 rng{ std::random_device{}() };

                array_uint_t indices;
                for (size_t i = 0; i < nparts; ++i) {
                    indices[i] = dist(rng);
                }

                return indices;
            }
        };

    }
} // namespace pf

#endif // MN_RESAMPLER_H
