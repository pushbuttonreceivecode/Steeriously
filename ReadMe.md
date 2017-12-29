<img src="steeriously.svg" width="400px" height="400px">

# Steeriously C++ Library

Steeriously. A dead-simple pure C++ library for adding steering behaviors to your game and multimedia entities.
All credit goes to Mat Buckland for writing "Programming AI by Example" and providing such great steering code.
All source is derived from "Programming AI by Example".
I can't recommend the book enough - you can buy the book here:
[Programming AI by Example](https://www.amazon.com/Programming-Example-Wordware-Developers-Library/dp/1556220782/ref=sr_1_1?ie=UTF8&qid=1512951593&sr=8-1&keywords=programming+ai+by+example)

Steeriously has one humble goal - to make steering behavior accessible and fun to program.
One way to achieve this is by redesigning the original code to expose a more general purpose API.
Tutorials and examples for using this library are available at:
[PushButtonReceiveCode.com](http://pushbuttonreceivecode.com) 

## Features include components for the following steering behaviors:

* Seek
* Flee
* Arrive
* Evade
* Pursuit
* Offset Pursuit (formation)
* Wander
* Interpose
* Alignment, Separation, and Cohesion (for flocking, or "emergent" behavior)
* Hiding
* Path Following (not path finding...that would be like A*, these are precalculated paths)
* Obstacle Avoidance
* Wall Avoidance
* Zero dependencies

It is also possible to extend the library by creating new components using
lower-level features. All steering functionality is provided in templated functions,
so derive your new components from steer::Agent and combine functions to create
your own steering component. You will need to override the following functions:

`virtual bool on(steer::behaviorType behavior) = 0;`

`virtual steer::Vector2 Calculate() = 0;`

..and probably these too, but they are not required:

`virtual bool accumulateForce(steer::Vector2 &startingForce, steer::Vector2 forceToAdd);`

`virtual steer::Vector2 calculateWeightedSum();`

`virtual steer::Vector2 calculatePrioritized();`

...then implement an update function passing in your frame time:

`void Update(float dt);`

The components I implemented should give you what you need to implement your own. There
are certainly things you can do much differently than what is provided in the example
components.

More information, tutorials, and demos along with API documentation are available at [pushbuttonreceivecode.com](http://pushbuttonreceivecode.com).

Steeriously does not use a build file. However, you can simply place the files in your project and add
include paths accordingly using your favorite IDE. Documentation will need built with Doxygen - just issue the doxygen command on the steeriously.doxy file via command line.