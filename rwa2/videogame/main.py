import character.woodman
import character.airman
import character.crashman
import character.hero

if __name__ == "__main__":
    """
    _summary_: Defines the hero and the bosses, then
               initiates the start of the game.

    """

    # Initialize hero
    megaman = character.hero.Hero("Mega Man")
    # Initialize Bosses
    woodman = character.woodman.WoodMan("WoodMan")
    airman = character.airman.AirMan("AirMan")
    crashman = character.crashman.CrashMan("CrashMan")
    bosses = [woodman, airman, crashman]

    # Begin fight with bosses
    megaman.fight(bosses)