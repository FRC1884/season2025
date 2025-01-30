# FRC 1884: REEFSCAPE Robot Code

**Overview** <br>
Welcome to FRC Team 1884 Griffins' repository for the 2025 FRC Season: REEFSCAPE! At the end of this season, this repository will be converted into a template that lives on this organization and can be drawn upon by the future generations of 1884, as well as our sister team 1797 Phoenix (currently inactive).

Starting this season, we've started doing things very differently than we have in the past. Feel free to explore the [ASL-Robotics](https://github.com/ASL-Robotics) and [FRC1884](https://github.com/frc1884) GitHub profiles to see our teams' repos from previous seasons!

**AdvantageKit** <br>
The most notable structural switch is that to the AdvantageKit logging paradigm. This has been created and maintained by Team 6328 Mechanical Advantage, with the intent to aid teams in the debugging process via a novel universal logging system. Check out their [2023 Championship Conference](https://www.youtube.com/watch?v=mmNJjKJG8mw) for more information on how it works.

An important part of this change has been moving towards hardware abstraction throughout our codebase. This allows us to swap between hardware implementations of our robot's various mechanisms with very little effort.

**More notable changes**
* The incorporation of [maple-sim](https://github.com/Shenzhen-Robotics-Alliance/maple-sim), courtesy of Team 5516 Iron Maple
  * This is a library that bundles the dyn4j physics engine into user code and allows us to simulate how our robot interacts with field and game elements. This will be exceedingly helpful for testing new field-dependent features and autonomous routines whenever we don't have a physical robot, such as during the prototyping and alpha construction stages.
* The creation of generic subsystems
  * A taster of the template nature of this repository in the future, these skeleton subsystems follow hardware abstraction and AdvantageKit principles, and represent the foundation of virtually any mechanism we will use. They can be easily subclassed and modified to fit the specifics of each mechanism without unnecessary boilerplate, and vastly expedite our experience testing prototypes faithfully to their intended behavior.
* State-based robot control
  * Abstraction is incredibly powerful, and this format allows us to employ a more declarative approach to robot behavior ("align and score game piece" v.s. "drive up, fine align, lift elevator...") that, in turn, opens up far more complex sequences of automated actions to decrease driver burden.
* *COMING SOON: System Identification*
* *COMING SOON: Vision in a Box*

**Credits** <br>
There are way too many people to thank and the list gets longer virtually every day, but we'd like to extend a massive thank you to every FRC team for publicizing their 2023-24 season repositories as inspiration for this project! Without your gracious professionalism, we would not have been able to get this massive project off the ground. Of course, we are also extremely grateful to team 6328 and everyone who has helped maintain the incredible framework that is AdvantageKit, as well as the extensive documentation that it thankfully has.

Here's a full list of the software libraries we are using, and the vendors accredited with them:
* Team 6328 Mechanical Advantage (AdvantageKit, AdvantageScope & URCL)
* SleipnirGroup (Choreo & ChoreoLib)
* Team 5516 Iron Maple (maple-sim)
* Team 3015 Ranger Robotics (PathPlanner & PathPlannerLib)
* CTR Electronics (Phoenix)
* PhotonVision (PhotonVision & PhotonLib)
* REV Robotics (REVlib)
* Studica (Studica NavX2)
* WPILib Suite (WPILib and the official FRC software ecosystem)

*(Updated 19 Jan)*
