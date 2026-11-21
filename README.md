# The system of point-to-point path mode for robot with discrete control

Golikov Kirill

Lomonosov MSU

2016

Key words: robotics, robotic arm, manipulator, positioning, path mode, gradient descent, machine learning


## ABSTRACT

The positioning algorithm of a robotic arm with discrete control is represented in this paper. 
The objective of the writing is a proof of concept of the manipulator positioning 
using “infinite” memory with no prior knowledge about the mathematical principles of the arm’s movement. 
The paper result is the algorithm of point-to-point path mode with several learning stages, 
the database of movements and the 2D customizable model of the manipulator with visualization.


## INTRODUCTION

This research make sure of possibility of the robot-manipulator positioning with a high precision inside a certain bounded area of the robot’s working space
without any analytical models of manipulator dynamics. Whereas that robotics now
is popular and booming branch of science and technics, the mass of various devices
appear every month. Producers want to conduct experiments and put into operation
these devices as soon as possible. The topicality of this paper is that it offers instead
of manual analytical calculations a way of automatic learning of the robot its own to
control its “extremities” by exploitation itself using the database of robot moves.


## BIOLOGICAL PREMISES

Effective moves of people by their extremities are result of complicated parallel computations going on in their brain, 
spinal cord and all path through the nerve cord directly up to the moving extremity. 
Moreover, the move uses the whole of aggregated great experience of previous movements of a human. 
Physiologists approve by their researches that our mind in fact does not calculate the plan and the implementation of the next move from the scratch. 
It rather assembles data of movements in our memory that fit in present case and, using these moves as blocks, it constructs the next move mechanically, without a thought [9]. 
Human brain is still detecting feedback from nerves and sense receptors in real time and it is correcting control that muscles are executing. 

Nobody wonder the fact that small children could overshoot the toy hanging above the bed, sometime they could even start the move in wrong direction. 
The same findings occur in robotics: the learning algorithm like a baby brain know nothing about itself and it explore its own organism structure and state acting its own extremities. 
Physiologists note that only after a long stage of learning baby could coordinate visual expectations with real moves.


## WHEREIN THE COMPLEXITY?

Robot dynamics establishes the motion laws, which are represented in systems of differential equations, 
and they connect the proactive forces of drives to inertial force, gravity, 
frictions and the other forces that are applied to the manipulator sections. 
In spite of that the formulae of mechanics and the methods of solving systems of differential equations well studied, 
the complexity to construct the adequate model is the finding values of the variety of equation parameters. 
It is needed to carry out calculations and to experiment on every robot individually. 
This way is laborious and expensive; its realization requires the qualified specialist.

In terms of unknown robot dynamics, every move degenerates into a multitude of misses and correcting turnings. 
This effect, besides, slows the execution phase, but also it could lead to collisions and damage of robot itself and manipulating objects. 
The proper realization of path finding and manipulator positioning imparts some “elegance” to a move: 
smoothness and quickness in reaching the required position, and a lack of misses. 
The extremities must be moving naturally, not only from the standpoint of similarity of human arm movements’ appearance, 
but also from the standpoint of performing operations effectiveness and coordination. 
It is necessary to create such algorithm that before a move it allows the robot to imagine where its extremity must be shifted 
and how to act from the start so that a deviation in the end of the positioning move would be within the determined precision.

Testing manipulator, it could understand the general principle of its operation. 
Having a certain set of trajectories that covers the whole of the robot working space; 
it could make conclusions about new trajectories. The solution is to specify iteratively the manipulator model by every its operation. 
Upon archiving the sample of training moves sufficient by volume and quality, the method to control the arm will be appear.


## PROBLEM STATEMENT

Let the robot-manipulator acts in its working space without obstacles and it has two rotatory joints: shoulder and elbow. 
A pneumatic engine provides the traveling of the robot’s grip along the plate. 
An individual pneumatic cylinder effects its own joint: open or close it corresponding the scheme of the valves. 
If as there are the two pairs of pairwise muscles-antagonists.

The **problem** is: *in terms of unknown physical model of the manipulator’s dynamics provide the optimal time of discovering a sets of such control actions,*
*which transfer the center of the grip to all defined points within some sub-region of robot’s working space with preassigned accuracy (= 1.5 mm)*. 
The **target** is such sub-region inside the robot’s working space, which contains all defined points.

This paper constructs the discrete model of the robot. “The special feature of discrete models is the discrete time in which functioning is performed” [7]. 
Such control actions of minimal long exist that shorter signals would not be “felt” by the manipulator if they have received. 
Besides the robot operates in continuous space but it could only produce the finite number of movements. 
Let fixate the minimal requiring precision and divide the space into regions in direct dependence on this precision. 
Let consider that the characteristic point of the manipulator’s grip is always exactly at the center of one of these regions. 
While the arm is moving, the grip is been shifting from a one region’s center to another.

In this paper the modeling task of manipulator positioning is “Go” game.

The robot’s target is the game field of square form with side equals to 30 sm (pic.1). 
The places of rock locating is the even lattice of 18x18 within the target, 324 points in all. 
The grip must hit these dedicated points. 
The target is completely situated in such place of the working space, where the manipulator could reach each its vertex.

![1](https://github.com/favorart/robot-hand/assets/9784285/ebe578bc-36b1-4662-adf0-91ab48409501)

Every movement is carried out from the **one origin point**.

**General control of the robot model** is $𝑣⃗(𝑡) = (𝑣_1(𝑡), 𝑣_2(𝑡), 𝑣_3(𝑡), 𝑣_4(𝑡))$,
the vector of binary coordinates, where each coordinate is a signal to correspond muscle in every moment of discrete time $t$.

The arm’s **muscle is acting** all the time while it is getting “unity” and it is being in an acceptable state. 
The muscle’s state is *acceptable*, when:

* the max (min) angle of opening (closing) has not reached;
* the pairwise muscle-antagonist is not active;
* the signal of immediate stop has not come.

If a signal has been changed from “unity” to “zero” then the muscle brakes an action, 
but the robotic arm continues to move by inertia. When the quiescent state has been reached, 
the robot signalizes it, after that the final point of a movement could be found out.

The manipulator has **feedback** that is the arm watches its own position, and this information is correct in each time.

Virtual manipulator models moves and tests algorithm correctness; 
it transfers the grip along the plane corresponding the combination of motion laws of all active muscles.

**Motion law** (*framing*) of the pairwise muscle is the value of shifting current angle of the joint in each frame of discrete time. 
The motion law realizes non-linear changings of velocity in time of muscle acting and simulating inertia.

![2](https://github.com/favorart/robot-hand/assets/9784285/20a7ddbc-2cd0-41d6-ad75-fde4e98f2ce1)

The motion laws of muscles with different constructions of drives and working regimes of engines are shown in the picture 2. 
In the picture 2, the increasing plots illustrate angle changings in time of engine working (of getting "unity"); 
the decreasing plots represent inertia.

**The movement trajectory** of a robotic arm is an ordered set of points that the grip has been visited in each frame of discrete time, 
while the move is performing from start to finish. 
The trajectory is characterized by its *final point*. The trajectories with the same final points are equivalent.

**The gradient descent** uses several trajectories whose final points are the closest to the aim point. 
The control actions of these trajectories are different from each other. 
The control duration of a next set-in could be varied a little in bounds of the control differences of the said trajectories. 
So, this move will be shifted along a direct line to the aim. Then this move’s stop should be closer to the aim point. 
Using the Taylor’s theorem the positioning algorithm could reach the aim point from a close enough point sequentially shifting the trajectory with the closest last point
along a direct line to the final in the aim.

**Taylor’s theorem**: if the rational function $f(х)$ has a derivative in point $a$, then it has a linear approximation in point $a$:

$$ 𝑓(𝑥) = 𝑓(𝑎) + 𝑓'(𝑎)(𝑥 − 𝑎) + h_1(𝑥 − 𝑎) \tag{1}$$

where $h_1(𝑥)$ is an error of approximation and $\lim_{𝑥 \to a}{h_1(𝑥)}=0$.

$$ 𝑃_1(𝑥) = 𝑓(𝑎) + 𝑓'(𝑎)(𝑥 − 𝑎) \tag{2}$$

where $𝑃_1(𝑥)$ is a linear approximation of function $𝑓(𝑥)$ in point $a$. 
The graphic $y=P_1(𝑥)$ is a tangent line to graphic of function $𝑓(𝑥)$ in point $𝑥=a$ [10].


## POSITIONING ALGORITHM

The algorithm consists of the three learning stages, after that the positioning becomes accurate.

At **the first learning stage**, the whole working space of the arm is being covered by movements 
from the origin point with some sufficiently large step of covering.

At that, the final points would not be located at the same distances from each other, 
because the robot does not know the exact acceleration functions of its muscles. 
The algorithm tries to keep about equal intervals between the final points of moves, 
its attempts are based on that the acceleration function does not decrease monotony. 
The covering is carried out by independent changings of the work duration of each robot muscle.

All trajectories performed would be written down into the database. 
Using the data, the first approximation of dependence of the grip position from control values is derived. 
Moreover, the arm configuration thresholds would be defined; 
those control thresholds would not take out the grip center from the target.

**The second learning stage** is the covering of the target by trajectories with
the less cover step more densely rather at the first stage.

**The third learning stage** is the database selection for trajectories with final points in some neighborhood of the aim point. 
The composition of the points’ controls produce the control for such sequential moves that next move is closer to stop in the aim point.

The well-learned robot could position its arm to a given location with the prescribed accuracy without any misses.


## THE THIRD LEARNING STAGE

1. If there is no supporting trajectories in an environment of the aim point, then it exits with an error.
2. While it is converging to the aim point, the averaged algorithm of gradient descent is working.
3. If the converging of the gradient is interrupted, i.e. it is in one of the local minimums of a 2D function 
different to the aim point. Than one of the “algorithm-impurity” will be used to “jump out of the hole” 
and to continue the process of gradient converging.
4. If the “algorithm-impurity” is realized, i.e. the point closer to aim has found then local minimum, 
then again it will return to averaged algorithm of gradient descent (step 2), else another “algorithm-impurity” will be used (step 3).
5. If “algorithm-impurity” is not realized, i.e. it continues to be in the local minimum, then to switch to basic gradient descent algorithm.
6. While it is converging to the aim point, the basic algorithm of gradient descent is working.
7. If the converging of the gradient is interrupted, then the exhaustive search of all directions will be used until the moment, when the aim has been reached.
8. If the exhaustive search has not succeed, that is the closest point is too far from the aim one. 
In this case, the conditions of Taylor’s theorem are not true. Exit from the algorithm.
9. Choose, instead of the aim, a random point in the square neighborhood (commensurable accuracy) of the aim point. 
Perform the described algorithm for the new point. If this point was closer to existing points, 
then the algorithm would converge to it. In terms of the new point’s selection, it is also close to the aim. 
Now, repeat the algorithm from this new point to the aim point. 
Either it would be converged and succeed or it would need to try another point. 

Practice had shown that to hit each fixated aim point, it is enough to choose a new point several times (< 15), 
where each third try is a check the aim point itself.


## RESULTS

**Complexity** of the algorithm is a general number of moves of the robot enough to hit each aim point.

**Method of time estimation**: the shoulder is being passed whole its path in 700 clocks of discrete time, and the elbow is in 550 clocks. 
Let one clock be one millisecond (1 ms). It is far from each movement is required the full opening or closing of a joint. 
One move of the arm performs in about a half of second, as much again it is needed to return the arm to the origin point. 
So, to set-in the grip there and back it spends *one second* of time in average.

The results of the described algorithm work with different parameters are followed in the tables 1-4. 
In these tables, the results of complexity and learning time are given for models with different motion laws:

### **MotionLaw**: *ContinuousAcceleration & ContinuousDeceleration*

Table 1

| Stage | Complexity | Covering step | Avg moves num | Aim points hits | Time        |
|:-----:|:----------:|:-------------:|:-------------:|:---------------:|:-----------:|
| 1     | 1359 moves | 18.75 mm      | 1             | 1.8%            | 23 minutes
| 2     | -          | -             | -             | -               | -
| 3     | 6796 moves | -             | 21            | 100%            | 114 minutes
| Total | 8155 moves |               |               | 100%            | 2.17 hours

Table 2

| Stage | Complexity | Covering step | Avg moves num | Aim points hits | Time        |
|:-----:|:----------:|:-------------:|:-------------:|:---------------:|:-----------:|
| 1     | 1359 moves | 18.75 mm      | 1             | 1.8%            | 23 minutes
| 2     | 4288 moves | 5.625 mm      | 1             | 28.7%           | 72 minutes
| 3     | 2973 moves | -             | 10            | 100%            | 50 minutes
| Total | 8620 moves |               |               | 100%            | 2.25 hours

### **MotionLaw**: *ContinuousSlowAcceleration & ContinuousDeceleration*

Table 3

| Stage | Complexity | Covering step | Avg moves num | Aim points hits | Time        |
|:-----:|:----------:|:-------------:|:-------------:|:---------------:|:-----------:|
| 1     | 1455 moves | 18.75 mm      | 1             | 0.3%            | 25 minutes
| 2     | -          | -             | -             | -               | -
| 3     | 4563 moves | -             | 14            | 100%            | 77 minutes
| Total | 6018 moves |               |               | 100%            | 1.42 hours

Table 4

| Stage | Complexity | Covering step | Avg moves num | Aim points hits | Time        |
|:-----:|:----------:|:-------------:|:-------------:|:---------------:|:-----------:|
| 1     | 1455 moves | 18.75 mm      | 1             | 0.3%            | 25 minutes
| 2     | 4217 moves | 5.625 mm      | 1             | 23.1%           | 71 minutes
| 3     | 2973 moves | -             | 5             | 100%            | 28 minutes
| Total | 8645 moves |               |               | 100%            | 2.7 hours

* **Avg moves num** is average quantity of moves to get one aim point.


## CONCLUSION

To prove the concept there was constructed the 2D model of the manipulator with discrete control that allows simulating different realizations of devices, 
also there was created the database of moves performed by the robot model and the positioning algorithm was programmed. 
This paper has shown that it is enough to use pre-learned trajectories to position the robot.

The writing’s result is also summary time estimation of the learning process, quantity of moves to cover the target with the prescribed accuracy, 
as well as evaluation of the database minimal volume enough for precise positioning. 
The learning algorithm succeed in archive all given points hit less than 15 moves in average. 
The whole learning of the algorithm requires approximately 2.5 hours.


## THE BIG GOAL

The task would not be reduced to effective control of the concrete robot. 
It is wanted to be able to use the most of learning results of one robot for speed-up learning of other similar manipulators. 
(In practice, manipulators of a one kind even fabricated by one technology by one machine tool could not have the absolute identity of positioning, 
because of the fine differences in fitting and assembling, which do not influence to their reliability). 
It is wanted to get the database of one robot included experience accumulated by others.


## REFERENCES

1. Russell S., Norvig P. - Artificial Intelligence: A Modern Approach. – 2nd ed. – New Jersey: Prentice Hall, 2003. – 985 c. – ISBN 0-13-790395-2.
2. ГОСТ 25686-85 Манипуляторы, автооператоры и пром. роботы. Термины и определения // М:ИПК Издательство стандартов, 1988. – 6 с. 10
3. ГОСТ 26050-89 Роботы промышленные. Общие технические требования // М:ИПК Издательство стандартов, 1995. – 16 с.
4. Андре П., Кофман Ж.-М., Лот Ф., Тайар Ж.П. - Конструирование роботов / пер. с фр. – М.: Мир, 1986. — 360 с., ил.
5. Бурдаков С.Ф. и др. - Проектирование манипуляторов промышленных роботов и роботизированных комплексов: Учеб. пособие для студ. вузов, обучающихся по спец. «Робототехнические системы» — М.: Высш. шк.,1986. — 264 с.: ил.
6. Василенко Н.В., Никитин К.Д., Пономарёв В.П., Смолин А.Ю. - Основы Робототехники / Под ред. Никитина К.Д. – Томск: "Радио и связь" МГП «РАСКО», 1993. — ISBN 5-88276-044-5.
7. Кудрявцев В.Б., Алешин С.В., Подколзин А.С. - Введение в теорию автоматов – М.: Наука. Гл. ред. физ.-мат. лит., 1985. – 320 с.
8. Никифоров С.О., Мархадаев Б.Е., Сумкин А.Г. - Экстремальные свойства стационарных движений двухшарнирного манипулятора // Изд. СО АН СССР, 1985. - №6. – Вып. .3. Сер. техн. наук,- 99 с.
9. Физиология человека: Учебник / Под ред. Покровского В.М., Коротько Г.Ф. - 2-е изд., перераб. и доп. - М.:Медицина, 2003. – 656 с.. ил. – (Учеб.лит. Для студ. мед. вузов). — ISBN 5-225-04729-7.
10. [Linear approximation - Wikipedia](https://en.wikipedia.org/wiki/Linear_approximation)
11. [Ильва Эллнеби. Моторные движения](https://web.archive.org/web/20220924062125/http://www.childneurologyinfo.com/sensory-text-breaking3.php)
12. [Тарабарин В.Б. Конспект лекций по курсу ТММ](https://web.archive.org/web/20200225212503/http://tmm-umk.bmstu.ru/lectures/)
13. [Golikov K.A. - Learning algorithm of systems with discrete control](http://intsysjournal.ru/pdfs/23-1/Golikov.pdf)
14. [Golikov K.A. - Learning systems with descrete control](http://intsysjournal.ru/pdfs/22-4/Golikov.pdf)


### SOURCE CODE

[The С++ source code is available on github](https://github.com/favorart/robot-hand/)

