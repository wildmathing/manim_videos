
"""
Сцена с лучом света внутри зеркального многоугольника.
Для зрителей канала Wild Mathing.
Видео: https://youtu.be/QDvZiy75x4Q
ManimCE v0.19.0: https://docs.manim.community/en/stable/index.html
Уроки для новичков: https://boosty.to/wildmathing/posts/7c3bdc8d-b048-41bb-b2a4-7355ef48f59b
"""


from manim import *


WILD_BLUE = ManimColor("#6699FF")
TracedPath.set_default(stroke_color=YELLOW, stroke_width=1.5)
DARK_DOT_KWARGS = {
    "radius": 0.04,
    "stroke_color": BLUE,
    "stroke_width": 1,
    "fill_color": BLACK,
    "fill_opacity": 1,
    "z_index": 1,
}
Dot.set_default(**DARK_DOT_KWARGS)


# Смена нормали при отражении луча
def reflect_vec(v, normal):
    return v - 2 * np.dot(v, normal) * normal


# Вычислить расстояние между точками
def get_distance(A, B):
    return np.linalg.norm(A - B)


# Поиск точки пересечения двух отрезков
def segment_intersection(p1, p2, q1, q2):
    p1, p2, q1, q2 = map(lambda x: x[:2], (p1, p2, q1, q2))
    d1 = p2 - p1
    d2 = q2 - q1
    det = d1[0] * (-d2[1]) + d1[1] * d2[0]

    if abs(det) < 1e-8:
        return None  # параллельны

    t = ((q1 - p1)[0] * (-d2[1]) + (q1 - p1)[1] * d2[0]) / det
    u = ((q1 - p1)[0] * (-d1[1]) + (q1 - p1)[1] * d1[0]) / det

    if 0 <= t <= 1 and 0 <= u <= 1:
        intersection = p1 + t * d1
        return np.array([intersection[0], intersection[1], 0])
    return None


# Функция, которая создает апдейтер для точки
def get_new_light(
        velocity,
        polygon,
        mob,
        vertex_epsilon=1e-5,
):
    edges = polygon.edges
    vertices = polygon.get_vertices()

    # Апдейтер точки
    def upd_dot(mob, dt):
        nonlocal velocity, edges, vertices

        # Останавливаем точки, попавшие в вершины
        for vertex in vertices:
            if get_distance(mob.get_center(), vertex) < vertex_epsilon:
                velocity = np.array((0, 0, 0))

        pos = mob.get_center()
        next_pos = pos + velocity * dt

        # Если еще не определено ребро пересечения, определяем его и нормаль
        if not hasattr(mob, "edge") or mob.edge is None:
            min_dist = np.inf
            for A, B in edges:
                pos_ray = pos + 30 * normalize(velocity)
                intersection = segment_intersection(pos, pos_ray, A, B)

                if intersection is not None:
                    new_dist = get_distance(pos, intersection)
                    if new_dist < min_dist:
                        edge = B - A
                        normal = np.array([-edge[1], edge[0], 0])
                        normal = normalize(normal)
                        mob.edge = [A, B]
                        mob.normal = normal
                        min_dist = new_dist

        # Если после шага пересечения с ребром нет, то переместим точку.
        inter = segment_intersection(pos, next_pos, mob.edge[0], mob.edge[1])
        if inter is None:
            mob.move_to(next_pos)
        else:
            dist = get_distance(pos, next_pos)
            dist_to_inter = get_distance(pos, inter)
            dist_to_new_pos = dist - dist_to_inter
            velocity = reflect_vec(velocity, mob.normal)
            end_pos = inter + dist_to_new_pos * velocity * dt
            mob.move_to(end_pos)
            mob.edge = None

    return upd_dot


# Зеркальный многоугольник
class MirrorPolygon(Polygon):
    def __init__(self, *vertices):
        super().__init__(*vertices)
        self.set_stroke(WILD_BLUE, 2.5)
        edges = [
            (np.array(vertices[i]), np.array(vertices[(i + 1) % len(vertices)]))
            for i in range(len(vertices))
        ]
        self.edges = edges


# Сцена с одним лучом
class ReflectingDot(Scene):
    def construct(self):
        # Многоугольник
        vertices = [
            [-1, -2, 0],
            [-1, 2, 0],
            [1, 1, 0],
            [3, -2, 0],
        ]
        polygon = MirrorPolygon(*vertices)
        dot = Dot(np.array([0, -1, 0]))
        velocity = np.array((2, 1, 0))
        tail = TracedPath(dot.get_center)
        self.add(polygon, dot, tail)

        # Апдейтер
        dot.add_updater(get_new_light(velocity, polygon, dot))
        for _ in range(3):
            self.wait(10)
