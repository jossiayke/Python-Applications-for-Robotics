from abc import ABC, abstractclassmethod
import character.hero
# import weapon.weapon
from weapon.weapon import Weapon


class Boss(ABC):
    """
    An abstract parent class which defines basic attributes
    of a boss that will be fighting the hero.

    Attributes:
        _name (str): name of the boss
        _hp (str): health level of the boss
        _life (int): number of lives for the boss, default to 1
        _is_defeated (bool): indicates whether boss is dead or alive
        _weapon (Weapon): takes in Weapon class which indicates type of weapon
                          boss carries to fight with hero
        _damage (int): indicates the amount of damage infliced by boss's weapon
        _is_idle (bool): shows if boss is idle (open to be attacked by hero)

    """

    def __init__(self, name: str, hp: float, weapon: Weapon):
        """
        Initialize Boss attributes.

        Args:
            name:str takes in name of the boss
            hp:float takes in the life level defined for the boss
            weapon:Weapon takes in the type of weapon defined for the boss

        Returns:
            None
        """
        self._name = name
        self._hp = hp
        self._life = 1
        self._is_defeated = False
        self._weapon = weapon
        self._damage = weapon._power
        self._is_idle = False

    @abstractclassmethod
    def use_special_ability(self, damage: float, hero=None):
        """
        Use this method to denote the special ability of each boss.

        Args:
            damage:float damage inflicted on boss by hero
            hero:Hero the hero who will be attacking or receiving the attack
                      from boss

        Returns:
            None
        """
        pass

    def take_damage(self, damage: float):
        """
        If Boss is idle, takes damage.

        Args:
            damage:float the amount of damage level boss will be receiving

        Returns:
            None

        """

        if self._hp > 0:
            self._hp -= damage
            print(
                f"{self._name} took {damage} damage points ({self._hp} HP left)")

        if self._hp <= 0:

            self._is_defeated = True

        self._is_idle = False

    def attack(self, hero: character.hero.Hero):
        """
        Boss Attacks Mega Man

        Args:
            hero:float the amount of damage level hero will be receiving

        Returns:
            None
        """

        print(f"*** {self._name} attacks ***")
        hero._take_damage(self._damage)

    def be_idle(self):
        """
        Set's Boss to be idle.

        Args:
            None

        Returns:
            None
        """

        self._is_idle = True
        print(f'{self._name} is idle')

    def __str__(self):
        """
        Displays the main attributes of boss

        Args:
            None

        Returns:
            Boss attributes: name, hp level, and weapon
        """

        return f'Name: {self._name}, HP: {self._hp}, Weapon: {self._weapon}'
