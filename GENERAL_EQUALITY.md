# MuJoCo 3.15: site-only GENERAL equality

Implemented against the supplied `mujoco-3.15.0(1).zip`. Existing equality enum values are unchanged;
`mjEQ_GENERAL` is appended as value 8. The supplied archive did not contain a HINGE equality type.

## XML API

```xml
<equality>
  <general name="connection" site1="linkA_site" site2="linkB_site"
           lock="1 1 1 0 0 0" torquescale="1 1 1"/>
</equality>
```

- Both site names and all six binary lock entries are required. At least one entry must be 1.
- Entries are `tx ty tz rx ry rz`, expressed in the moving site1 frame.
- Translation residual: `R1^T (p2-p1)`.
- Rotation residual: the vector part of `conj(q1)*q2`, multiplied componentwise by `torquescale`.
  This is `k*sin(phi/2)`, not three Euler angles. Locking one rotational component defines an
  orientation surface; it does not impose zero instantaneous angular velocity about that axis.
- The target of each selected component is zero. Initial offsets are not captured automatically.
- A site on worldbody supplies a world-fixed reference. No body, anchor, or relpose attributes.
- `torquescale` defaults to `1 1 1`. Its values must be finite and nonnegative, and strictly positive
  on locked rotational axes. The existing WELD torquescale remains a scalar.
- One shared `solref` and `solimp` applies to all selected rows, inherited through the existing
  equality defaults. Impedance uses the norm of the packed residual vector.
- Constraint enforcement remains soft. Quaternion residuals retain WELD's finite-angle limitations.

| Mask | Zero-residual geometry |
|---|---|
| `1 1 1 0 0 0` | Coincident site origins, free orientation |
| `1 1 1 1 1 1` | Coincident site frames |
| `1 1 1 1 1 0` | Hinge about the common z axis |
| `1 1 0 1 1 1` | Slide along site1 z |
| `1 1 0 1 1 0` | Slide and rotation along/about the common z axis |

These geometric equivalences do not promise identical finite-error soft dynamics to CONNECT/WELD.
The moving-frame derivative is included for translation, and the corresponding extra reference-body
moment is included in force reporting. The acceleration bias is analytic, not finite-differenced.

## C API and scope

For `mjsEquality`, set `type=mjEQ_GENERAL`, `objtype=mjOBJ_SITE`, and both names.
Initialize every mask entry in `data[0:6]` and every torque scale in `data[6:9]`; normally all scales
are 1. `data[9:11]` is reserved. This layout is also used in `mjModel.eq_data`, so `mjNEQDATA` stays 11.

The native C/C++ engine and MJCF parser/writer implement the feature. Python enum metadata and the
WebAssembly/Unity enum registrations are updated. Python, WebAssembly, and Unity packages were not
built here. MJX JAX/Warp constraint implementations are not added. Use a matching custom native
library and rebuilt bindings; an unmodified MuJoCo installation cannot load `<general>` equalities.

## Validation

Release C/C++ library and six affected test executables built successfully with GCC/G++.

| Suite | Result |
|---|---:|
| engine_core_constraint_test | 39 passed |
| engine_core_smooth_test | 69 passed |
| engine_island_test | 30 passed |
| engine_sleep_test | 20 passed |
| xml_native_reader_test | 160 passed |
| xml_native_writer_test | 78 passed, 1 skipped |
| test/doc/doc_test.py | 16 passed |
| **Total** | **412 passed, 1 skipped** |

The skipped writer test requires the unavailable `de_DE.UTF-8` locale. The writer suite was run
with the elasticity plugin built and `MUJOCO_PLUGIN_DIR` set to its directory.

The new tests exercise all 63 nonempty masks in dense and sparse modes with independent bodies
and a common moving ancestor (252 combinations). Sites are separated and rotated, torque scales
are unequal, and an unrelated DOF exercises sparse indexing. Tests compare residuals, central
finite-difference Jacobians, central finite-difference `Jdot*v`, and reported body wrenches against
`J^T lambda`. They also cover XML validation and round trips, MJB round trips, mixed equality row
packing, world references, activation, and waking a sleeping tree from a mocap connection.

The example `model/equality/general.xml` contains hinge, slider, and cylindrical connections. It
loaded and simulated for 250 steps with 14 equality rows and no simulation warnings.

## Build and run the focused tests

From the source root:

```bash
cmake -S . -B build-general \
  -DCMAKE_BUILD_TYPE=Release \
  -DMUJOCO_BUILD_TESTS=ON \
  -DMUJOCO_BUILD_EXAMPLES=OFF \
  -DMUJOCO_BUILD_SIMULATE=OFF \
  -DMUJOCO_TEST_PYTHON_UTIL=OFF
cmake --build build-general --target engine_core_constraint_test -j4
./build-general/bin/engine_core_constraint_test --gtest_filter='CoreConstraintTest.General*'
python3 -m pytest test/doc/doc_test.py -q
```

Use a fresh build directory when changing compiler or LTO settings. The full source archive includes
all changes and requires no patch application. Alternatively, from an untouched copy of the supplied
source, apply the accompanying patch with `patch -p1 < /path/to/general-equality.patch`.

## Modified files

- `doc/XMLreference.rst`
- `doc/XMLschema.rst`
- `doc/generate/generate_read_table.py`
- `doc/includes/references.h`
- `include/mujoco/mjtype.h`
- `python/mujoco/introspect/enums.py`
- `src/engine/engine_core_constraint.c`
- `src/engine/engine_core_smooth.c`
- `src/engine/engine_io.c`
- `src/engine/engine_island.c`
- `src/engine/engine_sleep.c`
- `src/engine/engine_vis_visualize.c`
- `src/user/user_model.cc`
- `src/user/user_objects.cc`
- `src/xml/generated/dmcontrol_schema.xml`
- `src/xml/generated/mjcf.xsd`
- `src/xml/generated/mjcf_map.h`
- `src/xml/generated/mjcf_table.inc`
- `src/xml/mjcf.schema`
- `src/xml/xml_native_reader.cc`
- `src/xml/xml_native_writer.cc`
- `test/engine/engine_core_constraint_test.cc`
- `unity/Runtime/Bindings/MjBindings.cs`
- `wasm/codegen/generated/bindings.cc`

## Added files

- `model/equality/general.xml` — runnable example.
- `GENERAL_EQUALITY.md` — this implementation and validation note.
