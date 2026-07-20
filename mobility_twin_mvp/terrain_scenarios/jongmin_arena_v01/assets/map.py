import math
import random
from pathlib import Path

import pychrono as chrono
import pychrono.irrlicht as chronoirr
import pychrono.vehicle as veh


# 1. 경기장 기본 파라미터
ARENA_LENGTH = 5.50
ARENA_WIDTH = 2.50
FLOOR_THICKNESS = 0.30

WALL_HEIGHT = 0.30
WALL_THICKNESS = 0.02

SAFE_ZONE_LENGTH = 0.50
ROCK_ZONE_LENGTH = 1.00
UNEVEN_ZONE_LENGTH = 1.00
OBSTACLE_ZONE_LENGTH = 1.00
SLOPE_ZONE_LENGTH = 1.00

TIME_STEP = 0.001

# 2. 암반지형 암석 개수
LARGE_ROCK_COUNT = 8
MEDIUM_ROCK_COUNT = 40
SMALL_ROCK_COUNT = 200

# 3. 암석 크기 범위 [m]
LARGE_ROCK_MIN_WIDTH = 0.18
LARGE_ROCK_MAX_WIDTH = 0.30

MEDIUM_ROCK_MIN_WIDTH = 0.10
MEDIUM_ROCK_MAX_WIDTH = 0.18

SMALL_ROCK_MIN_WIDTH = 0.035
SMALL_ROCK_MAX_WIDTH = 0.10

# 4. 암석 형상 및 배치 파라미터
ROCK_MIN_ASPECT_RATIO = 0.7
ROCK_MAX_ASPECT_RATIO = 1.5

ROCK_MIN_HEIGHT_RATIO = 0.35
ROCK_MAX_HEIGHT_RATIO = 0.9

ROCK_MIN_HEIGHT = 0.02
ROCK_MAX_HEIGHT = 0.15

ROCK_MIN_GAP = 0.0005

ROCK_MIN_EMBED_RATIO = 0.05
ROCK_MAX_EMBED_RATIO = 0.10

ROCK_CLUSTER_COUNT = 20

ROCK_CLUSTER_SIGMA_X = 0.15
ROCK_CLUSTER_SIGMA_Y = 0.22

ROCK_ANGLE_SEGMENTS = 12
ROCK_VERTICAL_LAYERS = 5

ROCK_NOISE_STRENGTH = 0.21
ROCK_NOISE_FREQUENCY = 3.0
ROCK_NOISE_OCTAVES = 3

MAX_PLACEMENT_ATTEMPTS_PER_ROCK = 10000

# 5. 암반 바닥 미세 요철 파라미터
ROUGH_TILE_SIZE = 0.10
ROUGH_MIN_HEIGHT = 0.002
ROUGH_MAX_HEIGHT = 0.014
ROUGH_NOISE_FREQUENCY = 2.6

# 6. 비평탄지형 파라미터
UNEVEN_GRID_SIZE = 0.05

UNEVEN_MIN_HEIGHT = -0.05
UNEVEN_MAX_HEIGHT = 0.08

UNEVEN_LARGE_WAVE_AMPLITUDE = 0.045
UNEVEN_MEDIUM_WAVE_AMPLITUDE = 0.030
UNEVEN_SMALL_NOISE_AMPLITUDE = 0.012

UNEVEN_NOISE_FREQUENCY = 3.5

UNEVEN_TRANSITION_LENGTH = 0.07

UNEVEN_MESH_BOTTOM_Z = -0.12

# 7. 대형 운석형 암석 파라미터
METEOR_WIDTH_X = 0.30
METEOR_WIDTH_Y = 0.30
METEOR_HEIGHT = 0.30

METEOR_ANGLE_SEGMENTS = 16
METEOR_VERTICAL_LAYERS = 7

METEOR_NOISE_STRENGTH = 0.26
METEOR_NOISE_FREQUENCY = 2.5

# 8. 장애물지형 파라미터
GATE_OPENING_WIDTH = 0.40
GATE_WALL_HEIGHT = 0.20
GATE_WALL_THICKNESS = 0.02

GATE_DISTANCE_FROM_ZONE_START = 0.05
GATE_SPACING_X = 0.45

# 경사 구조물 파라미터

# 경사로 통로 폭과 Y축 중심
# 경기장 중앙에서 -Y 방향으로 40 cm 이동
SLOPE_PATH_WIDTH = 1.00
SLOPE_PATH_CENTER_Y = 0.40

# 전반 0.5 m는 종경사, 후반 0.5 m는 횡경사
LONGITUDINAL_SECTION_LENGTH = 0.50
LATERAL_SECTION_LENGTH = 0.50

# 목표 경사각
LONGITUDINAL_SLOPE_ANGLE = 15.0
LATERAL_SLOPE_ANGLE = 12.0

# 횡경사가 서서히 생기고 사라지는 길이
SLOPE_TRANSITION_LENGTH = 0.10

# 경사 메시 격자 간격
SLOPE_GRID_SIZE = 0.025

# 경사 메시 밑면 높이
SLOPE_MESH_BOTTOM_Z = -0.12

# 경사로 마찰계수
SLOPE_FRICTION = 0.80

# SCM 느슨한 모래지형 파라미터

# 마지막 모래지형 구간 길이
PARTICLE_ZONE_LENGTH = 1.00

# 토양 표면에서 아래 지지 바닥까지 깊이
PARTICLE_SOIL_DEPTH = 0.05

# SCM 격자 간격
SCM_GRID_SPACING = 0.025

# 느슨한 비점착성 모래용 초기 시험값

# Bekker 압력-침하 계수
SCM_BEKER_KPHI = 1.5e5
SCM_BEKER_KC = 0.0
SCM_BEKER_N = 1.10

SCM_MOHR_COHESION = 0.0
SCM_MOHR_FRICTION_ANGLE = 28.0

SCM_JANOSI_SHEAR = 0.02

SCM_ELASTIC_STIFFNESS = 1.5e7
SCM_DAMPING = 2.0e4

# 9. 구간 좌표
#
# 진행 방향: -X → +X
#
# -2.75 ~ -2.25 : 평탄지형 0.5 m
# -2.25 ~ -1.25 : 암반지형 1.0 m
# -1.25 ~ -0.25 : 비평탄지형 1.0 m
# -0.25 ~ +0.75 : 장애물지형 1.0 m
# +0.75 ~ +2.75 : 남은 평탄구간 2.0 m

ARENA_X_MIN = -ARENA_LENGTH / 2.0
ARENA_X_MAX = ARENA_LENGTH / 2.0

SAFE_ZONE_X_MIN = ARENA_X_MIN
SAFE_ZONE_X_MAX = SAFE_ZONE_X_MIN + SAFE_ZONE_LENGTH

ROCK_ZONE_X_MIN = SAFE_ZONE_X_MAX
ROCK_ZONE_X_MAX = ROCK_ZONE_X_MIN + ROCK_ZONE_LENGTH

UNEVEN_ZONE_X_MIN = ROCK_ZONE_X_MAX
UNEVEN_ZONE_X_MAX = UNEVEN_ZONE_X_MIN + UNEVEN_ZONE_LENGTH

OBSTACLE_ZONE_X_MIN = UNEVEN_ZONE_X_MAX
OBSTACLE_ZONE_X_MAX = (
    OBSTACLE_ZONE_X_MIN
    + OBSTACLE_ZONE_LENGTH
)

# 장애물지형 다음에 경사구간 1 m
SLOPE_ZONE_X_MIN = OBSTACLE_ZONE_X_MAX
SLOPE_ZONE_X_MAX = (
    SLOPE_ZONE_X_MIN
    + SLOPE_ZONE_LENGTH
)

# 경사구간 뒤 남은 평탄 구간
REMAINING_ZONE_X_MIN = SLOPE_ZONE_X_MAX
REMAINING_ZONE_X_MAX = ARENA_X_MAX


def zone_center(x_min, x_max):
    return (x_min + x_max) / 2.0


SAFE_ZONE_CENTER_X = zone_center(
    SAFE_ZONE_X_MIN,
    SAFE_ZONE_X_MAX,
)

ROCK_ZONE_CENTER_X = zone_center(
    ROCK_ZONE_X_MIN,
    ROCK_ZONE_X_MAX,
)

UNEVEN_ZONE_CENTER_X = zone_center(
    UNEVEN_ZONE_X_MIN,
    UNEVEN_ZONE_X_MAX,
)

OBSTACLE_ZONE_CENTER_X = zone_center(
    OBSTACLE_ZONE_X_MIN,
    OBSTACLE_ZONE_X_MAX,
)

SLOPE_ZONE_CENTER_X = zone_center(
    SLOPE_ZONE_X_MIN,
    SLOPE_ZONE_X_MAX,
)

# 마지막 남은 1 m를 SCM 모래지형 구간으로 사용
PARTICLE_ZONE_X_MIN = REMAINING_ZONE_X_MIN
PARTICLE_ZONE_X_MAX = REMAINING_ZONE_X_MAX

PARTICLE_ZONE_CENTER_X = zone_center(
    PARTICLE_ZONE_X_MIN,
    PARTICLE_ZONE_X_MAX,
)

PARTICLE_ZONE_ACTUAL_LENGTH = (
    PARTICLE_ZONE_X_MAX
    - PARTICLE_ZONE_X_MIN
)

REMAINING_ZONE_LENGTH = (
    REMAINING_ZONE_X_MAX
    - REMAINING_ZONE_X_MIN
)

REMAINING_ZONE_CENTER_X = zone_center(
    REMAINING_ZONE_X_MIN,
    REMAINING_ZONE_X_MAX,
)

# 10. 노이즈 함수
def fade(value):
    return (
        value * value * value
        * (
            value
            * (value * 6.0 - 15.0)
            + 10.0
        )
    )

def lerp(value_a, value_b, ratio):
    return value_a + ratio * (value_b - value_a)

def hash_noise(ix, iy, iz, seed):
    value = (
        ix * 374761393
        + iy * 668265263
        + iz * 2147483647
        + seed * 1274126177
    )

    value = (
        value ^ (value >> 13)
    ) * 1274126177

    value = value ^ (value >> 16)

    normalized = (
        value & 0xFFFFFFFF
    ) / 0xFFFFFFFF

    return normalized * 2.0 - 1.0

def smooth_value_noise_2d(x, y, seed):
    x0 = math.floor(x)
    y0 = math.floor(y)

    x1 = x0 + 1
    y1 = y0 + 1

    tx = fade(x - x0)
    ty = fade(y - y0)

    n00 = hash_noise(x0, y0, 0, seed)
    n10 = hash_noise(x1, y0, 0, seed)
    n01 = hash_noise(x0, y1, 0, seed)
    n11 = hash_noise(x1, y1, 0, seed)

    nx0 = lerp(n00, n10, tx)
    nx1 = lerp(n01, n11, tx)

    return lerp(nx0, nx1, ty)

def smooth_value_noise_3d(x, y, z, seed):
    x0 = math.floor(x)
    y0 = math.floor(y)
    z0 = math.floor(z)

    x1 = x0 + 1
    y1 = y0 + 1
    z1 = z0 + 1

    tx = fade(x - x0)
    ty = fade(y - y0)
    tz = fade(z - z0)

    n000 = hash_noise(x0, y0, z0, seed)
    n100 = hash_noise(x1, y0, z0, seed)
    n010 = hash_noise(x0, y1, z0, seed)
    n110 = hash_noise(x1, y1, z0, seed)

    n001 = hash_noise(x0, y0, z1, seed)
    n101 = hash_noise(x1, y0, z1, seed)
    n011 = hash_noise(x0, y1, z1, seed)
    n111 = hash_noise(x1, y1, z1, seed)

    nx00 = lerp(n000, n100, tx)
    nx10 = lerp(n010, n110, tx)
    nx01 = lerp(n001, n101, tx)
    nx11 = lerp(n011, n111, tx)

    nxy0 = lerp(nx00, nx10, ty)
    nxy1 = lerp(nx01, nx11, ty)

    return lerp(nxy0, nxy1, tz)

def fractal_noise_3d(
    x,
    y,
    z,
    seed,
    octaves,
):
    total = 0.0
    amplitude = 1.0
    frequency = 1.0
    amplitude_sum = 0.0

    for octave in range(octaves):
        noise_value = smooth_value_noise_3d(
            x * frequency,
            y * frequency,
            z * frequency,
            seed + octave * 1009,
        )

        total += noise_value * amplitude
        amplitude_sum += amplitude

        amplitude *= 0.5
        frequency *= 2.0

    if amplitude_sum == 0.0:
        return 0.0

    return total / amplitude_sum

# 11. 고정 박스 생성
def create_fixed_box(
    system,
    size_x,
    size_y,
    size_z,
    position,
    material,
    color,
    name,
    collision=True,
):
    body = chrono.ChBodyEasyBox(
        size_x,
        size_y,
        size_z,
        2000.0,
        True,
        collision,
        material,
    )

    body.SetName(name)
    body.SetPos(position)
    body.SetFixed(True)

    if collision:
        body.EnableCollision(True)

    visual_shape = body.GetVisualShape(0)

    if visual_shape:
        visual_shape.SetColor(color)

    system.Add(body)

    return body

# 12. 구간별 바닥 생성
def create_segmented_floor(system):
    floor_material = chrono.ChContactMaterialSMC()
    floor_material.SetFriction(0.90)
    floor_material.SetRestitution(0.02)

    # 평탄 안전지형
    create_fixed_box(
        system,
        SAFE_ZONE_LENGTH,
        ARENA_WIDTH,
        FLOOR_THICKNESS,
        chrono.ChVector3d(
            SAFE_ZONE_CENTER_X,
            0.0,
            -FLOOR_THICKNESS / 2.0,
        ),
        floor_material,
        chrono.ChColor(0.31, 0.48, 0.29),
        "safe_zone_floor",
    )

    # 암반지형 기초
    create_fixed_box(
        system,
        ROCK_ZONE_LENGTH,
        ARENA_WIDTH,
        FLOOR_THICKNESS,
        chrono.ChVector3d(
            ROCK_ZONE_CENTER_X,
            0.0,
            -FLOOR_THICKNESS / 2.0,
        ),
        floor_material,
        chrono.ChColor(0.40, 0.24, 0.15),
        "rock_zone_base",
    )

    # 비평탄 메시 아래 지지 바닥
    support_thickness = (
        FLOOR_THICKNESS
        + UNEVEN_MESH_BOTTOM_Z
    )

    create_fixed_box(
        system,
        UNEVEN_ZONE_LENGTH,
        ARENA_WIDTH,
        support_thickness,
        chrono.ChVector3d(
            UNEVEN_ZONE_CENTER_X,
            0.0,
            UNEVEN_MESH_BOTTOM_Z
            - support_thickness / 2.0,
        ),
        floor_material,
        chrono.ChColor(0.42, 0.27, 0.17),
        "uneven_zone_support",
    )

    # 장애물지형 바닥
    create_fixed_box(
        system,
        OBSTACLE_ZONE_LENGTH,
        ARENA_WIDTH,
        FLOOR_THICKNESS,
        chrono.ChVector3d(
            OBSTACLE_ZONE_CENTER_X,
            0.0,
            -FLOOR_THICKNESS / 2.0,
        ),
        floor_material,
        chrono.ChColor(0.44, 0.38, 0.31),
        "obstacle_zone_floor",
    )


    # 경사 구조물 아래 지지 바닥
    #
    # 경사 메시의 일부는 횡경사 때문에 z=0 아래로 내려간다.
    # 따라서 통짜 평면을 z=0까지 올리지 않고
    # 메시 밑면 높이 아래에만 지지 바닥을 둔다.


    slope_support_thickness = (
        FLOOR_THICKNESS
        + SLOPE_MESH_BOTTOM_Z
    )

    create_fixed_box(
        system=system,
        size_x=SLOPE_ZONE_LENGTH,
        size_y=ARENA_WIDTH,
        size_z=slope_support_thickness,
        position=chrono.ChVector3d(
            SLOPE_ZONE_CENTER_X,
            0.0,
            SLOPE_MESH_BOTTOM_Z
            - slope_support_thickness / 2.0,
        ),
        material=floor_material,
        color=chrono.ChColor(
            0.41,
            0.36,
            0.30,
        ),
        name="slope_zone_support",
        collision=True,
    )    
    

    # SCM 아래 시각용 모래층
    # 실제 접촉은 SCM 표면이 담당한다.
    # 이 박스는 옆면이 비어 보이는 현상만 막으며 충돌하지 않는다.
    # 윗면을 z=-0.002 m로 내려 SCM 표면(z=0)과 겹치지 않게 한다.

    visual_fill_top_z = -0.002
    visual_fill_thickness = PARTICLE_SOIL_DEPTH

    create_fixed_box(
        system=system,
        size_x=PARTICLE_ZONE_ACTUAL_LENGTH,
        size_y=ARENA_WIDTH,
        size_z=visual_fill_thickness,
        position=chrono.ChVector3d(
            PARTICLE_ZONE_CENTER_X,
            0.0,
            visual_fill_top_z - visual_fill_thickness / 2.0,
        ),
        material=floor_material,
        color=chrono.ChColor(0.22, 0.12, 0.06),
        name="particle_zone_visual_fill",
        collision=False,
    )

# 13. 외곽벽 생성
def create_outer_walls(system):
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(0.80)
    material.SetRestitution(0.02)

    color = chrono.ChColor(
        0.78,
        0.78,
        0.80,
    )

    wall_z = WALL_HEIGHT / 2.0

    y_wall = (
        ARENA_WIDTH / 2.0
        + WALL_THICKNESS / 2.0
    )

    x_wall = (
        ARENA_LENGTH / 2.0
        + WALL_THICKNESS / 2.0
    )

    create_fixed_box(
        system,
        ARENA_LENGTH + 2.0 * WALL_THICKNESS,
        WALL_THICKNESS,
        WALL_HEIGHT,
        chrono.ChVector3d(0.0, y_wall, wall_z),
        material,
        color,
        "wall_positive_y",
    )

    create_fixed_box(
        system,
        ARENA_LENGTH + 2.0 * WALL_THICKNESS,
        WALL_THICKNESS,
        WALL_HEIGHT,
        chrono.ChVector3d(0.0, -y_wall, wall_z),
        material,
        color,
        "wall_negative_y",
    )

    create_fixed_box(
        system,
        WALL_THICKNESS,
        ARENA_WIDTH,
        WALL_HEIGHT,
        chrono.ChVector3d(x_wall, 0.0, wall_z),
        material,
        color,
        "wall_positive_x",
    )

    create_fixed_box(
        system,
        WALL_THICKNESS,
        ARENA_WIDTH,
        WALL_HEIGHT,
        chrono.ChVector3d(-x_wall, 0.0, wall_z),
        material,
        color,
        "wall_negative_x",
    )

# 14. 암반 바닥 미세 요철
def create_rough_rock_ground(system):
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(0.95)
    material.SetRestitution(0.01)

    seed = random.randint(
        0,
        2_000_000_000,
    )

    x_count = math.ceil(
        ROCK_ZONE_LENGTH / ROUGH_TILE_SIZE
    )

    y_count = math.ceil(
        ARENA_WIDTH / ROUGH_TILE_SIZE
    )

    tile_size_x = (
        ROCK_ZONE_LENGTH / x_count
    )

    tile_size_y = (
        ARENA_WIDTH / y_count
    )

    for ix in range(x_count):
        x = (
            ROCK_ZONE_X_MIN
            + tile_size_x * (ix + 0.5)
        )

        for iy in range(y_count):
            y = (
                -ARENA_WIDTH / 2.0
                + tile_size_y * (iy + 0.5)
            )

            noise = smooth_value_noise_2d(
                ix / x_count
                * ROUGH_NOISE_FREQUENCY,
                iy / y_count
                * ROUGH_NOISE_FREQUENCY,
                seed,
            )

            normalized = (
                noise + 1.0
            ) / 2.0

            height = (
                ROUGH_MIN_HEIGHT
                + (
                    ROUGH_MAX_HEIGHT
                    - ROUGH_MIN_HEIGHT
                )
                * normalized
            )

            color_offset = random.uniform(
                -0.02,
                0.02,
            )

            create_fixed_box(
                system,
                tile_size_x,
                tile_size_y,
                height,
                chrono.ChVector3d(
                    x,
                    y,
                    height / 2.0,
                ),
                material,
                chrono.ChColor(
                    0.42 + color_offset,
                    0.25 + color_offset,
                    0.16 + color_offset,
                ),
                f"rough_tile_{ix}_{iy}",
            )

# 15. 암석 표면점 생성
def generate_rock_points(
    width_x,
    width_y,
    total_height,
    noise_seed,
    angle_segments,
    vertical_layers,
    noise_strength,
    noise_frequency,
):
    points = chrono.vector_ChVector3d()

    half_x = width_x / 2.0
    half_y = width_y / 2.0

    for layer in range(vertical_layers + 1):
        layer_ratio = (
            layer / vertical_layers
        )

        polar_angle = (
            math.pi * layer_ratio
        )

        horizontal_factor = max(
            math.sin(polar_angle),
            0.20,
        )

        base_z = (
            total_height
            * (
                0.5
                - 0.5 * math.cos(polar_angle)
            )
        )

        for segment in range(angle_segments):
            angle = (
                2.0 * math.pi
                * segment
                / angle_segments
            )

            unit_x = (
                math.cos(angle)
                * horizontal_factor
            )

            unit_y = (
                math.sin(angle)
                * horizontal_factor
            )

            unit_z = (
                base_z / total_height
                if total_height > 0.0
                else 0.0
            )

            noise = fractal_noise_3d(
                unit_x * noise_frequency,
                unit_y * noise_frequency,
                unit_z * noise_frequency,
                noise_seed,
                ROCK_NOISE_OCTAVES,
            )

            radial_multiplier = (
                1.0
                + noise_strength * noise
            )

            shape_factor = (
                1.06
                - 0.10 * layer_ratio
            )

            px = (
                half_x
                * unit_x
                * radial_multiplier
                * shape_factor
            )

            py = (
                half_y
                * unit_y
                * radial_multiplier
                * shape_factor
            )

            pz = (
                base_z
                + noise
                * total_height
                * noise_strength
                * 0.12
            )

            pz = max(
                0.0,
                min(pz, total_height),
            )

            points.push_back(
                chrono.ChVector3d(
                    px,
                    py,
                    pz,
                )
            )

    return points

# 16. 일반 암석 생성
def create_rock(
    system,
    index,
    x,
    y,
    width_x,
    width_y,
    visible_height,
    embed_depth,
    yaw_deg,
    material,
):
    total_height = (
        visible_height + embed_depth
    )

    points = generate_rock_points(
        width_x,
        width_y,
        total_height,
        random.randint(
            0,
            2_000_000_000,
        ),
        ROCK_ANGLE_SEGMENTS,
        ROCK_VERTICAL_LAYERS,
        ROCK_NOISE_STRENGTH,
        ROCK_NOISE_FREQUENCY,
    )

    rock = chrono.ChBodyEasyConvexHull(
        points,
        2600.0,
        True,
        True,
        material,
    )

    rock.SetName(
        f"artificial_rock_{index:03d}"
    )

    rock.SetPos(
        chrono.ChVector3d(
            x,
            y,
            -embed_depth,
        )
    )

    rock.SetRot(
        chrono.QuatFromAngleZ(
            math.radians(yaw_deg)
        )
    )

    rock.SetFixed(True)
    rock.EnableCollision(True)

    shape = rock.GetVisualShape(0)

    if shape:
        shape.SetColor(
            chrono.ChColor(
                random.uniform(0.34, 0.47),
                random.uniform(0.21, 0.31),
                random.uniform(0.13, 0.21),
            )
        )

    system.Add(rock)

    return rock

# 17. 암석 겹침 검사
def is_position_valid(
    x,
    y,
    radius,
    placed_rocks,
):
    for old_x, old_y, old_radius in placed_rocks:
        dx = x - old_x
        dy = y - old_y

        minimum_distance = (
            radius
            + old_radius
            + ROCK_MIN_GAP
        )

        if (
            dx * dx + dy * dy
            < minimum_distance * minimum_distance
        ):
            return False

    return True

# 18. 암석 군집 생성
def create_cluster_centers():
    centers = []

    for _ in range(ROCK_CLUSTER_COUNT):
        centers.append(
            (
                random.uniform(
                    ROCK_ZONE_X_MIN + 0.05,
                    ROCK_ZONE_X_MAX - 0.05,
                ),
                random.uniform(
                    -ARENA_WIDTH / 2.0 + 0.05,
                    ARENA_WIDTH / 2.0 - 0.05,
                ),
            )
        )

    return centers

# 19. 암석 그룹 생성
def create_rock_group(
    system,
    start_index,
    count,
    min_width,
    max_width,
    placed_rocks,
    centers,
    material,
):
    current_index = start_index

    for _ in range(count):
        success = False

        for _ in range(
            MAX_PLACEMENT_ATTEMPTS_PER_ROCK
        ):
            base_width = random.uniform(
                min_width,
                max_width,
            )

            aspect = random.uniform(
                ROCK_MIN_ASPECT_RATIO,
                ROCK_MAX_ASPECT_RATIO,
            )

            width_x = (
                base_width * math.sqrt(aspect)
            )

            width_y = (
                base_width / math.sqrt(aspect)
            )

            visible_height = max(
                ROCK_MIN_HEIGHT,
                min(
                    base_width
                    * random.uniform(
                        ROCK_MIN_HEIGHT_RATIO,
                        ROCK_MAX_HEIGHT_RATIO,
                    ),
                    ROCK_MAX_HEIGHT,
                ),
            )

            embed_depth = (
                visible_height
                * random.uniform(
                    ROCK_MIN_EMBED_RATIO,
                    ROCK_MAX_EMBED_RATIO,
                )
            )

            radius = (
                max(width_x, width_y) / 2.0
            )

            if random.random() < 0.80:
                center_x, center_y = random.choice(
                    centers
                )

                x = random.gauss(
                    center_x,
                    ROCK_CLUSTER_SIGMA_X,
                )

                y = random.gauss(
                    center_y,
                    ROCK_CLUSTER_SIGMA_Y,
                )

            else:
                x = random.uniform(
                    ROCK_ZONE_X_MIN + radius,
                    ROCK_ZONE_X_MAX - radius,
                )

                y = random.uniform(
                    -ARENA_WIDTH / 2.0 + radius,
                    ARENA_WIDTH / 2.0 - radius,
                )

            if (
                x - radius < ROCK_ZONE_X_MIN
                or x + radius > ROCK_ZONE_X_MAX
                or y - radius < -ARENA_WIDTH / 2.0
                or y + radius > ARENA_WIDTH / 2.0
            ):
                continue

            if not is_position_valid(
                x,
                y,
                radius,
                placed_rocks,
            ):
                continue

            create_rock(
                system,
                current_index,
                x,
                y,
                width_x,
                width_y,
                visible_height,
                embed_depth,
                random.uniform(0.0, 360.0),
                material,
            )

            placed_rocks.append(
                (
                    x,
                    y,
                    radius,
                )
            )

            current_index += 1
            success = True

            break

        if not success:
            raise RuntimeError(
                "암석 배치 공간이 부족합니다. "
                "같은 설정으로 다시 실행하거나 "
                "암석 개수·크기를 줄여야 합니다."
            )

    return current_index

# 20. 암반지형 생성
def create_rock_zone(system):
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(0.95)
    material.SetRestitution(0.02)

    placed_rocks = []
    centers = create_cluster_centers()

    next_index = 1

    next_index = create_rock_group(
        system,
        next_index,
        LARGE_ROCK_COUNT,
        LARGE_ROCK_MIN_WIDTH,
        LARGE_ROCK_MAX_WIDTH,
        placed_rocks,
        centers,
        material,
    )

    next_index = create_rock_group(
        system,
        next_index,
        MEDIUM_ROCK_COUNT,
        MEDIUM_ROCK_MIN_WIDTH,
        MEDIUM_ROCK_MAX_WIDTH,
        placed_rocks,
        centers,
        material,
    )

    create_rock_group(
        system,
        next_index,
        SMALL_ROCK_COUNT,
        SMALL_ROCK_MIN_WIDTH,
        SMALL_ROCK_MAX_WIDTH,
        placed_rocks,
        centers,
        material,
    )

# 21. 비평탄지형 높이
def transition_weight(local_x):
    start_weight = min(
        local_x / UNEVEN_TRANSITION_LENGTH,
        1.0,
    )

    end_weight = min(
        (
            UNEVEN_ZONE_LENGTH - local_x
        )
        / UNEVEN_TRANSITION_LENGTH,
        1.0,
    )

    weight = max(
        0.0,
        min(
            min(start_weight, end_weight),
            1.0,
        ),
    )

    return fade(weight)

def calculate_uneven_height(
    local_x,
    y,
    noise_seed,
):
    normalized_x = (
        local_x / UNEVEN_ZONE_LENGTH
    )

    normalized_y = (
        y / ARENA_WIDTH + 0.5
    )

    large_wave = (
        math.sin(
            2.0
            * math.pi
            * (
                normalized_x * 1.25
                + normalized_y * 0.55
            )
        )
        * UNEVEN_LARGE_WAVE_AMPLITUDE
    )

    medium_wave = (
        math.sin(
            2.0
            * math.pi
            * (
                normalized_x * 2.1
                - normalized_y * 1.35
            )
            + 0.8
        )
        * UNEVEN_MEDIUM_WAVE_AMPLITUDE
    )

    small_noise = (
        smooth_value_noise_2d(
            normalized_x
            * UNEVEN_NOISE_FREQUENCY,
            normalized_y
            * UNEVEN_NOISE_FREQUENCY,
            noise_seed,
        )
        * UNEVEN_SMALL_NOISE_AMPLITUDE
    )

    height = (
        large_wave
        + medium_wave
        + small_noise
    )

    height *= transition_weight(local_x)

    return max(
        UNEVEN_MIN_HEIGHT,
        min(height, UNEVEN_MAX_HEIGHT),
    )

# 22. 비평탄 OBJ 메시 생성
def generate_uneven_terrain_obj():
    output_directory = (
        Path(__file__).resolve().parent
        / "generated_terrain"
    )

    output_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    output_path = (
        output_directory
        / "uneven_terrain.obj"
    )

    x_segments = round(
        UNEVEN_ZONE_LENGTH
        / UNEVEN_GRID_SIZE
    )

    y_segments = round(
        ARENA_WIDTH
        / UNEVEN_GRID_SIZE
    )

    x_points = x_segments + 1
    y_points = y_segments + 1

    noise_seed = random.randint(
        0,
        2_000_000_000,
    )

    vertices = []
    faces = []

    for ix in range(x_points):
        local_x = (
            UNEVEN_ZONE_LENGTH
            * ix / x_segments
        )

        world_x = (
            UNEVEN_ZONE_X_MIN + local_x
        )

        for iy in range(y_points):
            y = (
                -ARENA_WIDTH / 2.0
                + ARENA_WIDTH
                * iy / y_segments
            )

            z = calculate_uneven_height(
                local_x,
                y,
                noise_seed,
            )

            vertices.append(
                (
                    world_x,
                    y,
                    z,
                )
            )

    top_vertex_count = len(vertices)

    for ix in range(x_points):
        world_x = (
            UNEVEN_ZONE_X_MIN
            + UNEVEN_ZONE_LENGTH
            * ix / x_segments
        )

        for iy in range(y_points):
            y = (
                -ARENA_WIDTH / 2.0
                + ARENA_WIDTH
                * iy / y_segments
            )

            vertices.append(
                (
                    world_x,
                    y,
                    UNEVEN_MESH_BOTTOM_Z,
                )
            )

    def top_index(ix, iy):
        return (
            ix * y_points + iy + 1
        )

    def bottom_index(ix, iy):
        return (
            top_vertex_count
            + ix * y_points
            + iy
            + 1
        )

    # 윗면·밑면
    for ix in range(x_segments):
        for iy in range(y_segments):
            a = top_index(ix, iy)
            b = top_index(ix + 1, iy)
            c = top_index(ix + 1, iy + 1)
            d = top_index(ix, iy + 1)

            faces.append((a, b, c))
            faces.append((a, c, d))

            ba = bottom_index(ix, iy)
            bb = bottom_index(ix, iy + 1)
            bc = bottom_index(ix + 1, iy + 1)
            bd = bottom_index(ix + 1, iy)

            faces.append((ba, bb, bc))
            faces.append((ba, bc, bd))

    # -Y, +Y 측면
    for ix in range(x_segments):
        ta = top_index(ix, 0)
        tb = top_index(ix + 1, 0)
        ba = bottom_index(ix, 0)
        bb = bottom_index(ix + 1, 0)

        faces.append((ta, bb, tb))
        faces.append((ta, ba, bb))

        ta = top_index(ix, y_segments)
        tb = top_index(ix + 1, y_segments)
        ba = bottom_index(ix, y_segments)
        bb = bottom_index(ix + 1, y_segments)

        faces.append((ta, tb, bb))
        faces.append((ta, bb, ba))

    # 시작·끝 X 측면
    for iy in range(y_segments):
        ta = top_index(0, iy)
        tb = top_index(0, iy + 1)
        ba = bottom_index(0, iy)
        bb = bottom_index(0, iy + 1)

        faces.append((ta, tb, bb))
        faces.append((ta, bb, ba))

        ta = top_index(x_segments, iy)
        tb = top_index(x_segments, iy + 1)
        ba = bottom_index(x_segments, iy)
        bb = bottom_index(x_segments, iy + 1)

        faces.append((ta, bb, tb))
        faces.append((ta, ba, bb))

    with open(
        output_path,
        "w",
        encoding="utf-8",
    ) as obj_file:
        for x, y, z in vertices:
            obj_file.write(
                f"v {x:.8f} {y:.8f} {z:.8f}\n"
            )

        for a, b, c in faces:
            obj_file.write(
                f"f {a} {b} {c}\n"
            )

    return str(output_path), noise_seed

# 23. 비평탄 메시 강체 생성
def create_uneven_terrain(system):
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(0.92)
    material.SetRestitution(0.01)

    obj_path, noise_seed = (
        generate_uneven_terrain_obj()
    )

    terrain = chrono.ChBodyEasyMesh(
        obj_path,
        2000.0,
        True,
        True,
        True,
        material,
    )

    terrain.SetName(
        "uneven_terrain_mesh"
    )

    terrain.SetFixed(True)
    terrain.EnableCollision(True)

    shape = terrain.GetVisualShape(0)

    if shape:
        shape.SetColor(
            chrono.ChColor(
                0.48,
                0.29,
                0.17,
            )
        )

    system.Add(terrain)

    return noise_seed

# 경사 구조물 높이 계산
def smoothstep_01(value):
    """
    0~1 구간에서 시작과 끝의 기울기가 0인
    부드러운 전이 함수를 반환한다.
    """

    value = max(
        0.0,
        min(value, 1.0),
    )

    return (
        value * value
        * (3.0 - 2.0 * value)
    )

def calculate_longitudinal_height(local_x):
    """
    전반 0.5 m의 종경사 높이를 계산한다.

    0.00~0.25 m:
        완만하게 상승

    0.25~0.50 m:
        완만하게 하강

    시작점, 최고점, 종료점에서 기울기가 부드럽다.
    """

    if (
        local_x < 0.0
        or local_x > LONGITUDINAL_SECTION_LENGTH
    ):
        return 0.0

    half_section = (
        LONGITUDINAL_SECTION_LENGTH / 2.0
    )

    peak_height = (
        half_section
        * math.tan(
            math.radians(
                LONGITUDINAL_SLOPE_ANGLE
            )
        )
    )

    # sin² 곡선:
    # x=0에서 0
    # x=0.25에서 최고점
    # x=0.50에서 다시 0
    phase = (
        math.pi
        * local_x
        / LONGITUDINAL_SECTION_LENGTH
    )

    return (
        peak_height
        * math.sin(phase) ** 2
    )

def calculate_lateral_weight(local_x):
    """
    후반 0.5 m에서 횡경사가 서서히 생겼다가
    구간 끝에서 다시 사라지도록 가중치를 계산한다.

    후반 구간 기준:
        0.00~0.10 m: 0° → 12°
        0.10~0.40 m: 12° 유지
        0.40~0.50 m: 12° → 0°
    """

    lateral_x = (
        local_x
        - LONGITUDINAL_SECTION_LENGTH
    )

    if (
        lateral_x < 0.0
        or lateral_x > LATERAL_SECTION_LENGTH
    ):
        return 0.0

    if lateral_x < SLOPE_TRANSITION_LENGTH:
        ratio = (
            lateral_x
            / SLOPE_TRANSITION_LENGTH
        )

        return smoothstep_01(ratio)

    exit_start = (
        LATERAL_SECTION_LENGTH
        - SLOPE_TRANSITION_LENGTH
    )

    if lateral_x > exit_start:
        ratio = (
            LATERAL_SECTION_LENGTH
            - lateral_x
        ) / SLOPE_TRANSITION_LENGTH

        return smoothstep_01(ratio)

    return 1.0

def calculate_slope_height(local_x, y):
    """
    경사 구조물의 특정 X, Y 위치 높이를 반환한다.

    전반부:
        종경사 높이만 적용

    후반부:
        Y 방향 횡경사 적용
    """

    if local_x <= LONGITUDINAL_SECTION_LENGTH:
        return calculate_longitudinal_height(
            local_x
        )

    lateral_weight = (
        calculate_lateral_weight(local_x)
    )

    lateral_angle_radians = math.radians(
        LATERAL_SLOPE_ANGLE
    )

    # 경사로 중심을 기준으로 좌우 높이 계산
    lateral_height = (
        (y - SLOPE_PATH_CENTER_Y)
        * math.tan(lateral_angle_radians)
        * lateral_weight
    )

    return lateral_height

# 경사 구조물 OBJ 메시 생성
def generate_slope_terrain_obj():
    """
    종경사와 횡경사가 하나로 연결된 폐쇄형 OBJ 메시를 만든다.

    메시의 윗면:
        실제 로버 주행면

    메시의 밑면과 측면:
        폐쇄된 고정 강체 형상 구성
    """

    output_directory = (
        Path(__file__).resolve().parent
        / "generated_terrain"
    )

    output_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    output_path = (
        output_directory
        / "slope_terrain.obj"
    )

    x_segments = round(
        SLOPE_ZONE_LENGTH
        / SLOPE_GRID_SIZE
    )

    y_segments = round(
        SLOPE_PATH_WIDTH
        / SLOPE_GRID_SIZE
    )

    x_points = x_segments + 1
    y_points = y_segments + 1

    path_y_min = (
        SLOPE_PATH_CENTER_Y
        - SLOPE_PATH_WIDTH / 2.0
    )

    vertices = []
    faces = []


    # 윗면 꼭짓점


    for ix in range(x_points):
        local_x = (
            SLOPE_ZONE_LENGTH
            * ix / x_segments
        )

        world_x = (
            SLOPE_ZONE_X_MIN
            + local_x
        )

        for iy in range(y_points):
            y = (
                path_y_min
                + SLOPE_PATH_WIDTH
                * iy / y_segments
            )

            z = calculate_slope_height(
                local_x,
                y,
            )

            vertices.append(
                (
                    world_x,
                    y,
                    z,
                )
            )

    top_vertex_count = len(vertices)


    # 밑면 꼭짓점


    for ix in range(x_points):
        world_x = (
            SLOPE_ZONE_X_MIN
            + SLOPE_ZONE_LENGTH
            * ix / x_segments
        )

        for iy in range(y_points):
            y = (
                path_y_min
                + SLOPE_PATH_WIDTH
                * iy / y_segments
            )

            vertices.append(
                (
                    world_x,
                    y,
                    SLOPE_MESH_BOTTOM_Z,
                )
            )

    def top_index(ix, iy):
        return (
            ix * y_points
            + iy
            + 1
        )

    def bottom_index(ix, iy):
        return (
            top_vertex_count
            + ix * y_points
            + iy
            + 1
        )


    # 윗면과 밑면


    for ix in range(x_segments):
        for iy in range(y_segments):
            top_a = top_index(ix, iy)
            top_b = top_index(ix + 1, iy)
            top_c = top_index(ix + 1, iy + 1)
            top_d = top_index(ix, iy + 1)

            faces.append(
                (top_a, top_b, top_c)
            )

            faces.append(
                (top_a, top_c, top_d)
            )

            bottom_a = bottom_index(ix, iy)
            bottom_b = bottom_index(ix, iy + 1)
            bottom_c = bottom_index(
                ix + 1,
                iy + 1,
            )
            bottom_d = bottom_index(
                ix + 1,
                iy,
            )

            faces.append(
                (
                    bottom_a,
                    bottom_b,
                    bottom_c,
                )
            )

            faces.append(
                (
                    bottom_a,
                    bottom_c,
                    bottom_d,
                )
            )


    # -Y 측면


    for ix in range(x_segments):
        top_a = top_index(ix, 0)
        top_b = top_index(ix + 1, 0)

        bottom_a = bottom_index(ix, 0)
        bottom_b = bottom_index(ix + 1, 0)

        faces.append(
            (top_a, bottom_b, top_b)
        )

        faces.append(
            (top_a, bottom_a, bottom_b)
        )


    # +Y 측면


    last_y = y_segments

    for ix in range(x_segments):
        top_a = top_index(ix, last_y)
        top_b = top_index(ix + 1, last_y)

        bottom_a = bottom_index(ix, last_y)
        bottom_b = bottom_index(ix + 1, last_y)

        faces.append(
            (top_a, top_b, bottom_b)
        )

        faces.append(
            (top_a, bottom_b, bottom_a)
        )


    # 시작 X 측면


    for iy in range(y_segments):
        top_a = top_index(0, iy)
        top_b = top_index(0, iy + 1)

        bottom_a = bottom_index(0, iy)
        bottom_b = bottom_index(0, iy + 1)

        faces.append(
            (top_a, top_b, bottom_b)
        )

        faces.append(
            (top_a, bottom_b, bottom_a)
        )


    # 끝 X 측면


    last_x = x_segments

    for iy in range(y_segments):
        top_a = top_index(last_x, iy)
        top_b = top_index(last_x, iy + 1)

        bottom_a = bottom_index(last_x, iy)
        bottom_b = bottom_index(last_x, iy + 1)

        faces.append(
            (top_a, bottom_b, top_b)
        )

        faces.append(
            (top_a, bottom_a, bottom_b)
        )


    # OBJ 파일 저장


    with open(
        output_path,
        "w",
        encoding="utf-8",
    ) as obj_file:
        obj_file.write(
            "# Generated longitudinal and lateral slope terrain\n"
        )

        for x, y, z in vertices:
            obj_file.write(
                f"v {x:.8f} {y:.8f} {z:.8f}\n"
            )

        for a, b, c in faces:
            obj_file.write(
                f"f {a} {b} {c}\n"
            )

    return str(output_path)

# 경사 구조물 강체 생성
def create_slope_terrain(system):
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(SLOPE_FRICTION)
    material.SetRestitution(0.01)

    obj_path = generate_slope_terrain_obj()

    slope_body = chrono.ChBodyEasyMesh(
        obj_path,
        2000.0,
        True,
        True,
        True,
        material,
    )

    slope_body.SetName(
        "combined_slope_terrain"
    )

    slope_body.SetFixed(True)
    slope_body.EnableCollision(True)

    visual_shape = slope_body.GetVisualShape(0)

    if visual_shape:
        visual_shape.SetColor(
            chrono.ChColor(
                0.50,
                0.42,
                0.31,
            )
        )

    system.Add(slope_body)

    return slope_body

# 마지막 1 m SCM 느슨한 모래지형 생성
def create_particle_terrain(system):
    """마지막 1 m 구간에 SCM 기반 변형 가능 토양을 생성한다."""

    terrain = veh.SCMTerrain(system)

    # 경사 지형 끝점과 SCM 시작면을 모두 z=0에 맞춘다.
    terrain.SetReferenceFrame(
        chrono.ChCoordsysd(
            chrono.ChVector3d(
                PARTICLE_ZONE_CENTER_X,
                0.0,
                0.0,
            ),
            chrono.ChQuaterniond(
                1.0,
                0.0,
                0.0,
                0.0,
            ),
        )
    )

    terrain.Initialize(
        PARTICLE_ZONE_ACTUAL_LENGTH,
        ARENA_WIDTH,
        SCM_GRID_SPACING,
    )

    terrain.SetSoilParameters(
        SCM_BEKER_KPHI,
        SCM_BEKER_KC,
        SCM_BEKER_N,
        SCM_MOHR_COHESION,
        SCM_MOHR_FRICTION_ANGLE,
        SCM_JANOSI_SHEAR,
        SCM_ELASTIC_STIFFNESS,
        SCM_DAMPING,
    )

    # 침하량·압력 컬러맵을 끈다.
    terrain.SetPlotType(
        veh.SCMTerrain.PLOT_NONE,
        0.0,
        1.0,
    )

    # 기본 어두운 갈색
    terrain.SetColor(
        chrono.ChColor(
            0.22,
            0.12,
            0.06,
        )
    )

    # 실제 존재하는 Chrono 흙 텍스처 적용
    terrain.SetTexture(
        chrono.GetChronoDataFile(
            "vehicle/terrain/textures/dirt.jpg"
        ),
        4.0,
        10.0,
    )

    terrain.SetMeshWireframe(False)

    return terrain

# 24. 대형 운석형 암석 생성
def create_meteor_rock(
    system,
    name,
    x,
    y,
    ground_z,
):
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(0.95)
    material.SetRestitution(0.02)

    points = generate_rock_points(
        METEOR_WIDTH_X,
        METEOR_WIDTH_Y,
        METEOR_HEIGHT,
        random.randint(
            0,
            2_000_000_000,
        ),
        METEOR_ANGLE_SEGMENTS,
        METEOR_VERTICAL_LAYERS,
        METEOR_NOISE_STRENGTH,
        METEOR_NOISE_FREQUENCY,
    )

    meteor = chrono.ChBodyEasyConvexHull(
        points,
        2800.0,
        True,
        True,
        material,
    )

    meteor.SetName(name)

    # 매립하지 않고 지면 위에 그대로 배치
    meteor.SetPos(
        chrono.ChVector3d(
            x,
            y,
            ground_z,
        )
    )

    meteor.SetRot(
        chrono.QuatFromAngleZ(
            random.uniform(
                0.0,
                2.0 * math.pi,
            )
        )
    )

    meteor.SetFixed(True)
    meteor.EnableCollision(True)

    shape = meteor.GetVisualShape(0)

    if shape:
        shape.SetColor(
            chrono.ChColor(
                0.35,
                0.26,
                0.20,
            )
        )

    system.Add(meteor)

    return meteor

# 25. 암반·비평탄 구간 대형 암석 배치
def create_large_meteor_rocks(
    system,
    uneven_noise_seed,
):
    """
    암반지형에는 랜덤 위치로 운석 1개를 배치한다.

    비평탄지형의 운석은 게이트 진입 방향 앞 중앙에
    고정 배치하여 로버가 우회한 뒤 게이트로 진입하게 한다.
    """

    meteor_radius = (
        max(
            METEOR_WIDTH_X,
            METEOR_WIDTH_Y,
        )
        / 2.0
    )

    # 1. 암반지형 운석
    rock_x = random.uniform(
        ROCK_ZONE_X_MIN + meteor_radius,
        ROCK_ZONE_X_MAX - meteor_radius,
    )

    rock_y = random.uniform(
        -ARENA_WIDTH / 2.0 + meteor_radius,
        ARENA_WIDTH / 2.0 - meteor_radius,
    )

    # 암반 타일은 최대 ROUGH_MAX_HEIGHT까지 올라오므로
    # 그 표면 위에 운석을 올린다.
    create_meteor_rock(
        system=system,
        name="meteor_rock_zone",
        x=rock_x,
        y=rock_y,
        ground_z=ROUGH_MAX_HEIGHT,
    )


    # 2. 비평탄지형 운석

    # 게이트 위치:
    #   OBSTACLE_ZONE_X_MIN + 0.05
    #
    # 운석 위치:
    #   게이트보다 0.20 m 앞


    gate_x = (
        OBSTACLE_ZONE_X_MIN
        + GATE_DISTANCE_FROM_ZONE_START
    )

    uneven_x = gate_x - 0.50
    uneven_y = 0.0

    local_x = (
        uneven_x - UNEVEN_ZONE_X_MIN
    )

    ground_z = calculate_uneven_height(
        local_x,
        uneven_y,
        uneven_noise_seed,
    )

    create_meteor_rock(
        system=system,
        name="meteor_uneven_zone",
        x=uneven_x,
        y=uneven_y,
        ground_z=ground_z,
    )

def create_gate(
    system,
    gate_x,
    gate_center_y,
    gate_index,
    material,
    color,
):
    """
    경기장 폭 전체를 벽으로 막고,
    지정한 Y 위치에 폭 40 cm의 입구를 만든다.
    """

    arena_y_min = -ARENA_WIDTH / 2.0
    arena_y_max = ARENA_WIDTH / 2.0

    opening_y_min = (
        gate_center_y
        - GATE_OPENING_WIDTH / 2.0
    )

    opening_y_max = (
        gate_center_y
        + GATE_OPENING_WIDTH / 2.0
    )

    negative_wall_length = (
        opening_y_min - arena_y_min
    )

    positive_wall_length = (
        arena_y_max - opening_y_max
    )

    negative_wall_center_y = (
        arena_y_min
        + negative_wall_length / 2.0
    )

    positive_wall_center_y = (
        opening_y_max
        + positive_wall_length / 2.0
    )

    wall_center_z = (
        GATE_WALL_HEIGHT / 2.0
    )

    # 입구 아래쪽(-Y 방향) 벽
    create_fixed_box(
        system=system,
        size_x=GATE_WALL_THICKNESS,
        size_y=negative_wall_length,
        size_z=GATE_WALL_HEIGHT,
        position=chrono.ChVector3d(
            gate_x,
            negative_wall_center_y,
            wall_center_z,
        ),
        material=material,
        color=color,
        name=f"gate_{gate_index}_negative_wall",
        collision=True,
    )

    # 입구 위쪽(+Y 방향) 벽
    create_fixed_box(
        system=system,
        size_x=GATE_WALL_THICKNESS,
        size_y=positive_wall_length,
        size_z=GATE_WALL_HEIGHT,
        position=chrono.ChVector3d(
            gate_x,
            positive_wall_center_y,
            wall_center_z,
        ),
        material=material,
        color=color,
        name=f"gate_{gate_index}_positive_wall",
        collision=True,
    )

# 26. 장애물지형 게이트와 기둥
def create_obstacle_zone(system):
    material = chrono.ChContactMaterialSMC()
    material.SetFriction(0.85)
    material.SetRestitution(0.02)

    gate_color = chrono.ChColor(
        0.72,
        0.71,
        0.68,
    )



    # 게이트 X 좌표
    #
    # 게이트 1: 장애물 구간 시작 후 5 cm
    # 게이트 2: 게이트 1에서 45 cm 뒤
    # 게이트 3: 게이트 2에서 45 cm 뒤


    gate_1_x = (
        OBSTACLE_ZONE_X_MIN
        + GATE_DISTANCE_FROM_ZONE_START
    )

    gate_2_x = (
        gate_1_x
        + GATE_SPACING_X
    )

    gate_3_x = (
        gate_2_x
        + GATE_SPACING_X
    )


    # 게이트 1
    # 입구 중앙: y = 0.00 m


    create_gate(
        system=system,
        gate_x=gate_1_x,
        gate_center_y=0.00,
        gate_index=1,
        material=material,
        color=gate_color,
    )


    # 게이트 2
    # 게이트 1보다 우측(+Y) 40 cm


    create_gate(
        system=system,
        gate_x=gate_2_x,
        gate_center_y=-0.40,
        gate_index=2,
        material=material,
        color=gate_color,
    )


    # 게이트 3
    # 게이트 2보다 다시 우측(+Y) 40 cm
    # 첫 번째 게이트 기준 총 80 cm 이동


    create_gate(
        system=system,
        gate_x=gate_3_x,
        gate_center_y=0.4,
        gate_index=3,
        material=material,
        color=gate_color,
    )

# 27. 메인 함수
def main():
    system = chrono.ChSystemSMC()

    system.SetCollisionSystemType(
        chrono.ChCollisionSystem.Type_BULLET
    )

    system.SetGravitationalAcceleration(
        chrono.ChVector3d(0.0, 0.0, -9.81)
    )

    chrono.ChCollisionModel.SetDefaultSuggestedEnvelope(0.0025)
    chrono.ChCollisionModel.SetDefaultSuggestedMargin(0.0025)

    # 경기장과 구간별 지형
    create_segmented_floor(system)
    create_outer_walls(system)

    create_rough_rock_ground(system)
    create_rock_zone(system)

    uneven_noise_seed = create_uneven_terrain(system)
    create_large_meteor_rocks(system, uneven_noise_seed)

    create_obstacle_zone(system)
    create_slope_terrain(system)

    # 마지막 1 m SCM 지형
    create_particle_terrain(system)

    # Irrlicht 시각화
    vis = chronoirr.ChVisualSystemIrrlicht()
    vis.AttachSystem(system)
    vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
    vis.SetWindowSize(1280, 720)
    vis.SetWindowTitle(
        "K-SRC Arena - Rock, Uneven, Gate, Slope and SCM Terrain"
    )
    vis.Initialize()

    vis.AddLogo(
        chrono.GetChronoDataFile("logo_chrono_alpha.png")
    )

    vis.AddSkyBox()

    vis.AddCamera(
        chrono.ChVector3d(-4.8, -4.5, 3.8),
        chrono.ChVector3d(-0.2, 0.0, 0.0),
    )

    vis.AddLightWithShadow(
        chrono.ChVector3d(0.0, 0.0, 7.0),
        chrono.ChVector3d(0.0, 0.0, 0.0),
        8.0,
        2.0,
        12.0,
        45.0,
        1024,
    )

    realtime_timer = chrono.ChRealtimeStepTimer()

    while vis.Run():
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        system.DoStepDynamics(TIME_STEP)
        realtime_timer.Spin(TIME_STEP)


if __name__ == "__main__":
    main()