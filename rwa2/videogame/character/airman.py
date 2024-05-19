from character.boss import Boss
from weapon.weapon import Weapon
# from character.hero import Hero
from random import randint


class AirMan(Boss):
    """
    A derived class, child of Boss, which defines basic attributes
    of a Airman boss that will be fighting the hero.

    Attributes:
        _dodge_odd (float): odds that boss can
                                    dodge from an attack


    """

    def __init__(self, name: str = "AirMan", dodge_odd: int = 6):
        """
        Initialize Airman attributes.

        Args:
            name:str takes in name of Airman as assigned
            dodge_odd:int takes in the odd of dodging an attack

        Returns:
            None
        """
        super().__init__(name, hp=250, weapon=Weapon("AirShooter", 25))
        # print(self._name)
        self._dodge_odd = dodge_odd

    def use_special_ability(self, damage: int, hero=None):
        """
        Airman uses this ability to dodge attacks from hero and avoid damage.

        Args:
            damage:float damage inflicted on boss by hero
            hero:Hero the hero who will be attacking or receiving the attack
                      from boss

        Returns:
            None
        """

        result = randint(1, self._dodge_odd)
        if not result == self._dodge_odd:

            super().take_damage(damage)
            # print("Airman is idle")
            # print(f'****{self._name} dodged the attack***')

        else:
            print("*** Airman dodges the attack ***")
