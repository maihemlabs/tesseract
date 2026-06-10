# Plan: TSDF / SDF collision support in Tesseract via `btSdfCollisionShape`

## Context

Tesseract has no volumetric signed-distance-field geometry. The existing `SDF_MESH`
type is misleadingly named — it is just a `PolygonMesh` subtype, not a distance field.
We want true SDF collision: a baked distance field (generated offline from a mesh) that
Bullet can use for discrete contact checks and signed-distance/gradient queries.

Bullet 3.25 (already installed at `/opt/homebrew/.../BulletCollision/CollisionShapes/btSdfCollisionShape.h`)
ships `btSdfCollisionShape`. Its **only** data-ingestion path is:

```cpp
bool initializeSDF(const char* sdfData, int sizeInBytes);
```

which consumes a serialized **btMiniSDF blob** (the Discregrid `.sdf`/`.cdf` cubic-interpolant
format) — *not* a raw dense distance grid. It also exposes
`queryPoint(const btVector3& pt, btScalar& dist, btVector3& normal)`.

**Decisions confirmed with user:**
- **Data model:** store the serialized btMiniSDF byte blob and pass it straight through
  to `initializeSDF` (no in-house dense-grid → btMiniSDF conversion).
- **Scope:** Bullet **discrete** collision + **distance queries**. No continuous/cast
  support (the cast manager is convex-only; `btSdfCollisionShape` is concave). No FCL parity.
- **Source:** SDF baked offline from a mesh, loaded at runtime as a resource/blob.

Naming: user calls it "TSDF". The blob is a full (not necessarily truncated) Discregrid SDF,
so we name the geometry class `SDF` / enum `SDF` for accuracy, but this is open to a `TSDF`
rename if preferred (see open question at bottom).

---

## Part A — New geometry type (`tesseract_geometry`)

Model it on `Octree`, which is the existing volumetric type that serializes a binary blob.

**New files**
- `geometry/include/tesseract/geometry/impl/sdf.h` — class `SDF : public Geometry`
- `geometry/src/geometries/sdf.cpp`

**Class shape** (mirrors `octree.h`):
```cpp
class SDF : public Geometry {
  // construct from serialized btMiniSDF bytes, or from a resource (.sdf file)
  SDF(std::vector<std::uint8_t> sdf_data,
      Eigen::Vector3d scale = {1,1,1},
      double margin = 0.0);
  SDF() = default;

  const std::vector<std::uint8_t>& getData() const;   // serialized blob
  const Eigen::Vector3d& getScale() const;            // -> btSdfCollisionShape::setLocalScaling
  double getMargin() const;                           // -> setMargin

  Geometry::Ptr clone() const override final;
  bool operator==/!=(const SDF&) const;
private:
  std::vector<std::uint8_t> sdf_data_;
  Eigen::Vector3d scale_{1,1,1};
  double margin_{0.0};
  template <class Archive> friend void serialize(...);
};
```
Constructor passes `GeometryType::SDF` to the `Geometry` base (see `box.cpp` pattern).
Optionally accept a `tesseract_common::Resource::ConstPtr` overload that reads the
`.sdf` file bytes (same Resource pattern used by `PolygonMesh`), for URDF/file loading.

**Wiring (follow the Octree precedent exactly):**
1. `geometry/include/tesseract/geometry/geometry.h` — add `SDF` to the `GeometryType`
   enum and `"SDF"` to `GeometryTypeStrings` (keep both in lockstep — they're index-aligned).
2. `geometry/include/tesseract/geometry/fwd.h` — forward-declare `class SDF;`.
3. `geometry/include/tesseract/geometry/geometries.h` — `#include impl/sdf.h`.
4. `geometry/include/tesseract/geometry/cereal_serialization.h` — add `serialize(Archive&, SDF&)`;
   serialize the blob as binary (copy the `make_binary_data` approach used for the Octree blob).
5. `geometry/include/tesseract/geometry/cereal_serialization_impl.hpp` — add
   `CEREAL_REGISTER_TYPE(...::SDF)` and `CEREAL_REGISTER_POLYMORPHIC_RELATION(Geometry, SDF)`.
6. `geometry/CMakeLists.txt` — add `src/geometries/sdf.cpp` to the `geometry` library sources.

**Other `GeometryType` switch sites in `tesseract_geometry`** (must add an `SDF` case so
they don't hit the default/error path):
- `geometry/src/conversions.cpp`
- `geometry/src/utils.cpp` (e.g. bounding-box / volume helpers — derive AABB from the
  blob's stored domain if available, otherwise return a conservative/empty result and log).

---

## Part B — Bullet backend (`tesseract_collision_bullet`)

**File:** `collision/bullet/src/bullet_utils.cpp`

1. `#include <BulletCollision/CollisionShapes/btSdfCollisionShape.h>`.
2. Add an overload near the other `createShapePrimitive(...)` overloads:
   ```cpp
   std::shared_ptr<BulletCollisionShape>
   createShapePrimitive(const tesseract::geometry::SDF::ConstPtr& geom) {
     auto sdf = std::make_shared<btSdfCollisionShape>();
     const auto& d = geom->getData();
     if (!sdf->initializeSDF(reinterpret_cast<const char*>(d.data()),
                             static_cast<int>(d.size())))
       throw std::runtime_error("Failed to initialize btSdfCollisionShape from blob");
     sdf->setLocalScaling(convertEigenToBt(geom->getScale()));
     auto out = std::make_shared<BulletCollisionShape>();
     out->top_level = sdf;          // children stays empty
     return out;
   }
   ```
3. Add the `case GeometryType::SDF:` to the dispatch `switch` (lines ~371-435).
   Set margin via `shape->top_level->setMargin(...)` using `geom->getMargin()` (or
   `BULLET_MARGIN` as default), matching the surrounding cases.

**Distance queries:** discrete contact + distance flows through the manager's
`btCollisionDispatcher`. `btSdfCollisionShape` is a `btConcaveShape`; verify the
default `btDefaultCollisionConfiguration` registers the convex-vs-SDF narrowphase
algorithm so `contactTest`/distance works. **Risk item** — if not auto-registered we
must call `dispatcher->registerCollisionCreateFunc(...)` (or use the SDF-aware near
callback) in `BulletDiscreteBVHManager`/`BulletDiscreteSimpleManager` setup. Confirm
during implementation with a unit test before declaring done.

**Cast/continuous:** explicitly out of scope. `makeCastCollisionObject()` already throws
for non-convex shapes; ensure the error is comprehensible (SDF is concave). No code change
beyond possibly a clearer message.

**FCL backend:** out of scope. `fcl_utils.cpp` will hit its default-error case for `SDF`;
acceptable for now (document that SDF requires the Bullet backend).

---

## Part C — (Optional, can be a follow-up) loading & tooling

- **URDF**: add a `<sdf>` geometry parser (mirror `urdf/src/sdf_mesh.cpp`) that takes a
  `filename` resource pointing at a baked `.sdf` blob. Skip if programmatic construction
  is enough for v1.
- **Baking utility**: document the offline workflow to produce the btMiniSDF blob from a
  mesh (Bullet's `obj2sdf` / Discregrid `GenerateSDF`). No code, just README/docs.
- **Visualization** (`visualization/src/ignition/conversions.cpp`): add an `SDF` case
  (render its AABB or skip with a warning) so the switch is exhaustive.

---

## Verification

1. **Build:** `cmake --build` the `geometry` and `collision_bullet` targets.
2. **Geometry unit test** (`geometry/test/tesseract_geometry_unit.cpp`): construct an `SDF`
   from a small baked blob fixture, assert `getType()==SDF`, `clone()`, `operator==`, and
   round-trip through Cereal (JSON + binary archives) — copy the Octree test pattern.
3. **Collision unit test** (new test in `collision/bullet/test/`): bake a `.sdf` from a
   simple box mesh offline, add it as a collision object in `BulletDiscreteBVHManager`,
   place a sphere overlapping it, and assert `contactTest` returns a contact with a
   plausible signed distance (negative when penetrating) — this validates both discrete
   contact and the distance/gradient path, and flushes out the narrowphase-registration risk.
4. Confirm a cast manager rejects `SDF` with a clear error (negative test).

## Open questions / risks
- **Name:** `SDF` vs `TSDF` for the class/enum (using `SDF` for accuracy; easy to rename).
- **Narrowphase registration risk** (Part B) — the one genuine unknown; gated by test #3.
- **Bullet version:** `btSdfCollisionShape` requires Bullet ≥ ~2.89; the repo's
  `find_bullet()` already enforces double precision. May want a CMake feature-check that
  the header exists and `#define` a `TESSERACT_BULLET_HAS_SDF` guard.
