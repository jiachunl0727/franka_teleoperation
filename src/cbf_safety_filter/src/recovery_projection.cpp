#include "cbf_safety_filter/recovery_projection.hpp"
namespace cbf_safety_filter {
Vec7 recovery_projection(const ConstraintSet& rows, const Vec7& nominal, const TorqueBounds& box,
                         const Vec7& weights, int sweeps) noexcept {
  const auto clip = [&](const Vec7& x) -> Vec7 {
    return x.cwiseMax(box.lower).cwiseMin(box.upper);
  };
  const auto project = [&](bool requested) {
    Vec7 value = clip(nominal);
    for (int sweep = 0; sweep < sweeps; ++sweep) {
      bool changed = false;
      for (int i = 0; i < rows.count; ++i) {
        const auto& row = rows.rows[i];
        const double limit = requested ? rows.requested_bounds[i] : row.bound;
        const double violation = row.g.dot(value) - limit;
        if (violation > 0) {
          const Vec7 direction = row.g.cwiseQuotient(weights);
          const double norm = row.g.dot(direction);
          if (norm > 1e-24) {
            value = clip(value - direction * (violation / norm));
            changed = true;
          }
        }
      }
      if (!changed)
        break;
    }
    return value;
  };
  // Prefer the unmodified requested inequalities. Do not weaken a table or
  // joint row just because the generic braking anchor violates it: another
  // feasible corrective torque may exist. Only use recovery when this bounded
  // first pass cannot certify the complete requested intersection.
  Vec7 candidate = project(true);
  bool requested_ok = candidate.allFinite();
  for (int i = 0; requested_ok && i < rows.count; ++i)
    requested_ok = rows.rows[i].g.dot(candidate) <= rows.requested_bounds[i] + 1e-10;
  if (requested_ok)
    return candidate;
  candidate = project(false);
  if (!candidate.allFinite())
    return rows.braking_anchor;
  const Vec7 direction = candidate - rows.braking_anchor;
  double fraction = 1;
  for (int i = 0; i < rows.count; ++i) {
    const auto& row = rows.rows[i];
    const double delta = row.g.dot(direction);
    if (delta > 0)
      fraction =
          std::min(fraction, std::max(0., row.bound - row.g.dot(rows.braking_anchor)) / delta);
  }
  if (fraction < 1)
    fraction *= 1 - 1e-10;
  return clip(rows.braking_anchor + fraction * direction);
}
}  // namespace cbf_safety_filter
