from dataclasses import dataclass


@dataclass
class Weapon:
    _name: str
    _power: float

    def __str__(self):
        return f'Name: {self._name}, x: {self._power}'

# class Weapon():
#     def __init__(self, name, power):
#         self._name = name
#         self._power = power

#     @property
#     def name(self):
#         return self._name

#     @name.setter
#     def name(self, name):
#         self._name = name

#     @property
#     def power(self):
#         return self._power

#     # raise a custom exception with a more elaborate and specific message
#     @power.setter
#     def power(self, power):
#         self._power = power
