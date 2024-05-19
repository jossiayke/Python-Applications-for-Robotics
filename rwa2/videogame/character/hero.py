from typing import List
from random import randint
# from weapon.weapon import Weapon
import weapon.weapon
import character.boss


class Hero():
    """
    A class which defines basic attributes
    of a hero that will be fighting the boss.

    Attributes:
        name (str): name of the hero
        _initial_hp (int): amount that will initialize hero life 
                           at start of duel
        hp (str): health level of the hero
        weapon (Weapon): takes in Weapon class which indicates type of weapon
                          boss carries to fight with hero
        damage (int): indicates the amount of damage infliced by boss's weapon
        boss_now (Boss): the current boss fighting hero
        is_defeated (bool): indicates whether hero is dead or alive
        _jump_chance (int): chance (odds) upto which hero will be jumping
                            an attack, ie from random range from 1
                            to jump_chance
        _is_idle (bool): shows if hero is idle (open to be attacked by hero)

    """

    def __init__(self, name: str):
        """
        Initialize Hero attributes.

        Args:
            name:str takes in name of the hero

        Returns:
            None
        """

        self.name = name
        self._initial_hp = 200
        self._hp = self._initial_hp
        self._weapon = weapon.weapon.Weapon("MegaBlade", 15)
        self._damage = self._weapon._power
        self._boss_now = character.boss.Boss
        self._is_defeated = False
        self._jump_chance = 2
        self._is_idle = False

    @property
    def name(self):
        return self._name

    @name.setter
    def name(self, name):
        self._name = name

    def _attack(self):
        """
        Hero attacks boss after setting boss to idle. 
        Verifies if hero is defeated.

        Args:
            hero:float the amount of damage level boss will be receiving

        Returns:
            None
        """

        self._boss_now.be_idle()
        print(f'*** {self._name} attacks ***')
        self._boss_now.use_special_ability(self._damage, self)
        self._boss_now._is_idle = False

        if self._hp <= 0:
            self._is_defeated = True

    def _jump(self):
        """
        Hero jumps to avoid attack from boss. 

        Args:
            None

        Returns:
            None
        """

        if randint(1, self._jump_chance) == self._jump_chance:
            print(f'**** {self._name} jumps ***')
            return True
        return False

    def _take_damage(self, damage: float):
        """
        If Hero didn't jump an attack, takes damage.

        Args:
            damage:float the amount of damage level hero will be receiving

        Returns:
            None

        """

        if (not self._jump()):
            if self._hp > 0:
                self._hp -= damage
                print(
                    f"{self._name} took {damage} damage points ({self._hp} HP left)")

            if self._hp <= 0:
                # print(
                #     f"{self._name} took {damage} damage points ({self._hp} HP left) and died")
                self._is_defeated = True

    def _get_boss_weapon(self, boss=None):
        """
        If Boss is defeated, hero takes the boss's weapon and has his
        health reset

        Args:
            boss:Boss the boss whom hero has defeated

        Returns:
            None

        """

        self._weapon = boss._weapon
        self._initial_hp = self._initial_hp + 50
        self._hp = self._initial_hp
        print(f"*** Weapon {self._weapon._name} has been acquired ***")
        print()

    def fight(self, bosses: List):
        """
        The sequence of actions occuring while hero faces the bosses.
        The battlefield where all the defined characters come meet
        Lists the steps of the game which lead the Hero from start
        to finish line or until the hero dies

        Args:
            bosses:List[Boss] a list of all the bosses involved in game who
                              are listed to face the hero

        Returns:
            None

        """

        for boss in bosses:
            self._boss_now = boss

            # Begin fight with boss
            print("----------------------------------------")
            print(f"*** FIGHTING {self._boss_now._name}***  ")
            print("----------------------------------------")
            print()
            # (self._damage)]

            while ((not self._is_defeated) and
                    (not self._boss_now._is_defeated)):
                "While Hero is still alive resume with game"

                boss_actions = [self._boss_now.attack,
                                self._boss_now.use_special_ability]

                action = boss_actions[randint(0, 1)]

                if action == self._boss_now.attack:
                    self._boss_now.attack(self)
                else:
                    self._attack()

            if self._boss_now._is_defeated:
                print("--------------------------------------------")
                print(f"{self._boss_now._name} has been defeated    ")
                print("--------------------------------------------")
                print()
                self._get_boss_weapon(self._boss_now)

            else:
                break

        if self._is_defeated:
            print()
            print(
                "*******************************************************************************")
            print(
                f"{self._name} has been defeated by {self._boss_now._name}....GAME OVER")
            print(
                "*******************************************************************************")

            print()
            print(f" That's why {self._boss_now._name} is the Boss... X)")
            print()
            print("Better luck next time...")

        else:
            print()
            print(
                "*******************************************************************************")
            print("Congratulations! All bosses have been defeated.")
            print(
                "*******************************************************************************")

            print()
            print(f"What would we have done without you {self._name}... :) ")
            print()
            print("Good Job")
