# Orrobot
Orrobot team code for robockey competition

This project aims to create autonomous hockey playing robots using a Atmega328U board.
This was a class project to design and build autonomous robots that play hockey on a rink and follow start/stop commands sent over 2.4GHz rf. There were strict requirements for size but the rest was open, using the <a href="http://medesign.seas.upenn.edu/index.php/Guides/MaEvArM">MAEVARM board</a>, an ATMEGA38U4 board built specifically for C development in this course at the University of Pennsylvania by Jonathan Fiene.


![](https://github.com/sjono/Orrobot/raw/master/img/0701_orrobot_overview.png)

The three separate robot hockey players ran separate code, Attacker, Enforcer and Goalie, which referred to m_robockey.h - this library included the main localization code to read a constellation above the bots and determine their location on the rink.


![](https://github.com/sjono/Orrobot/raw/master/img/0701_robockey.png)

