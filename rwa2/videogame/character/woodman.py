from character.boss import Boss
from weapon.weapon import Weapon
# from character.hero import Hero
from random import randint


class WoodMan(Boss):
    """
    A derived class, child of Boss, which defines basic attributes
    of a Woodman boss that will be fighting the hero.

    Attributes:
        _attack_resistance (float): amount in percentage that boss can 
                                    resist from an attack
        _resistance_odd (float): chance of resisting an attack by hero


    """

    def __init__(self, name: str = "WoodMan", resistance_odd: int = 4):
        """
        Initialize WoodMan attributes.

        Args:
            name:str takes in name of WoodMan as assigned 
            resistance_odd:int takes in the odd of resisting an attack

        Returns:
            None
        """

        super().__init__(name, hp=200, weapon=Weapon("LeafShield", 20))
        # print(self._name)
        self._attack_resistance = 0.5
        self._resistance_odd = resistance_odd

    def use_special_ability(self, damage: int, hero=None):
        """
        Denotes the ability of Woodman to resist an attack an receive a less 
        severe damage from hero.

        Args:
            damage:float damage inflicted on boss by hero
            hero:Hero the hero who will be attacking or receiving the attack
                      from boss

        Returns:
            None
        """

        result = randint(1, self._resistance_odd)
        if result == self._resistance_odd:
            print("Woodmand resists the attack")
            super().take_damage(damage/self._attack_resistance)

        else:

            super().take_damage(damage)
