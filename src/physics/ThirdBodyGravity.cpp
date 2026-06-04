#include "physics/ThirdBodyGravity.h"
#include "environment/SunModel.h"
#include "environment/MoonModel.h"
#include "core/math/Constants.h"

ThirdBodyGravity::ThirdBodyGravity(ThirdBodyType body, double epoch_jd)
    : body_(body)
    , epoch_jd_(epoch_jd)
{
    mu_ = (body == ThirdBodyType::Sun) ? Constants::GM_SUN : Constants::GM_MOON;
}

Vec3 ThirdBodyGravity::acceleration(const OrbitState&         state,
                                     const PhysicalProperties& /*props*/,
                                     double                    time_s) const
{
    const double jd = epoch_jd_ + time_s / Constants::SEC_PER_DAY;

    // Position of the third body in ECI [m]
    const Vec3 r_b = (body_ == ThirdBodyType::Sun)
                         ? SunModel::position_eci(jd)
                         : MoonModel::position_eci(jd);

    // Vector from satellite to third body
    const Vec3   d      = r_b - state.position;
    const double d_norm = d.norm();
    const double rb     = r_b.norm();

    // Perturbation: direct term minus indirect (Earth's own acceleration toward body)
    return mu_ * (d / (d_norm * d_norm * d_norm) - r_b / (rb * rb * rb));
}
