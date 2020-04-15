import math
from numbers import Number


class Vector2D():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    @classmethod  # Alternative constructor
    def from_scalars(cls, angle, direction):
        x = direction * math.cos(angle)
        y = direction * math.sin(angle)
        return cls(x, y)

    @property
    def direction(self):
        return math.atan2(self.y, self.x)

    @property
    def magnitude(self):
        return (self.x**2 + self.y**2)**0.5

    @property
    def mag_squared(self):
        return (self.x**2 + self.y**2)

    def normalized(self):
        return self / self.magnitude

    def rotate(self, rad):
        return Vector2D(
            math.cos(rad) * self.x - math.sin(rad) * self.y,
            math.sin(rad) * self.x + math.cos(rad) * self.y)

    # Arithmetic Operations #

    def __add__(self, other):
        if not other: return self
        return Vector2D(self.x + other.x, self.y + other.y)

    def __radd__(self, other):
        return self.__add__(other)

    def __sub__(self, other):
        return self + -other

    def __mul__(self, scalar):
        if not isinstance(scalar, Number):
            raise NotImplementedError('NotImplemented: Vector2D cannot be multiplied by non-scalar!')
        return Vector2D(self.x * scalar, self.y * scalar)

    def __truediv__(self, scalar):
        return self.__mul__(1 / scalar)

    def __mod__(self, tup):
        x, y = tup
        return Vector2D(self.x % x, self.y % y)

    # Unary Operations #

    def __neg__(self):
        return Vector2D(-self.x, -self.y)

    def __str__(self):
        return f"Vector2D(x: {self.x}, y: {self.y})"

    # Sequence Methods #

    def __len__(self):
        return 2

    def __getitem__(self, i):
        # return self.y if i else self.x
        return (self.x, self.y)[i]
