from character.boss import Boss
from weapon.weapon import Weapon
# from character.hero import Hero
from random import randint


class CrashMan(Boss):
    """
    A derived class, child of Boss, which defines basic attributes
    of a CrashMan boss that will be fighting the hero.

    Attributes:
        _reflection_odd (float): odds that boss can
                                reflect an attack sent to it back to the hero


    """

    def __init__(self, name: str = "CrashMan", reflection_odd: int = 5,
                 reflected_percentage: float = 10):
        """
        Initialize CrashMan attributes.

        Args:
            name:str takes in name of CrashMan as assigned
            reflection_odd:int takes in the odd of reflecting an attack

        Returns:
            None
        """
        super().__init__(name, hp=300, weapon=Weapon("CrashBomber", 30))
        # print(self._name)
        self._reflection_odd = reflection_odd
        self._reflected_percentage = reflected_percentage

    def use_special_ability(self, damage: int, hero=None):
        """
        Crashman uses this ability to reflect attack back to hero and
        avoid damage.

        Args:
            damage:float damage inflicted on boss by hero
            hero:Hero the hero who will be attacking or receiving the attack
                      from boss

        Returns:
            None
        """

        result = randint(1, self._reflection_odd)
        if result == self._reflection_odd:
            reflected_damage = (self._reflected_percentage/100) * damage
            hero._take_damage(reflected_damage)
            print(f"CrashMan reflects {reflected_damage} damage")

        else:

            super().take_damage(damage)
