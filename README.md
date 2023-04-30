# Final Challenge Part A: Lane Following

This is 2023 RSS Team 14's repo for the final challenge part A.

### TODO

1. Coordinate node published/subscribed messages
2. Account for lane detection errors: store last known lane location? map lanes in world frame? idk
3. Add homography function or node
4. Change pure_pursuit topics/inputs (no longer using published trajectory)
5. Fix safety controller error on car
6. Make sure safety controller doesn't stop car when close to other cars
7. What is a Hough transform and should we use it?

##

<p align="center">Below here is info from the original <a href="https://github.com/mit-rss/final_challenge2023">final challenge repo</a>!</p>

##

## Deliverables & Grading

| Deliverable | Due Date              |
|---------------|----------------------------------------------------------------------------|
| Race Day | Saturday, May 13th 11AM - 2PM EST |
| Code Pushed to Github  | Saturday, May 13th 1PM EST |
| Briefing (15 min presentation + 5 min Q&A) OR Report ([github pages](https://github.mit.edu/rss/website2021)) | Wednesday, May 10th at 1:00PM EST (NB: deadline is for briefing slides, briefings are from 3-5pm) |
| [Team Member Assessment](https://forms.gle/5npgrmk8mjdRGGcL7)  | Wednesday, May 10th at 11:59PM EST |

##

| Deliverable  Grade | Weighting             |
|---------------|----------------------------------------------------------------------------|
| Part A: Final Race (out of 100)  | 35% |
| Part B: City Driving  | 25% |
| Part C: Rainbow Road [BONUS]  | 10% |
| Briefing OR Report Grade (out of 10) | 40% |

## Part A: Final Race
Part A is worth 35% of your Final Challenge technical grade. Your grade will be calculated based on the time your car takes to drive around the track (`best_race_split`, in seconds) as follows:

  `Part A grade = min(100 + (50 - best_race_split), 110)  - penalties`

Where `penalties` is calculated as follows:

  `penalties = 15 * num_collisions + 5 * num_lane_line_breaches + 5 * num_long_breaches`
  
And `num_lane_line_breaches` is the number of times the car drives outside of either lane line, and `num_long_breaches` is the number of times the car has driven outside of its lane and stayed outside of the lane for greater than 3 seconds.

As you can see from this grading scheme, it is possible to receive bonus points for a very fast and precise solution. The **maximum speed of your car should be capped at 4 m/s**; you should be able to get full points (with bonus!) with a good controller. You should, above all, prioritize avoiding collisions, and if your car leaves its lane, it should quickly recover. More information about race day can be found below in this handout.

### Environment and Task

The Final Race will take place on the entire Johnson track loop. This is a standard-size 200m track. Cars may be assigned to follow any of the track's six lanes and will be informed of their lane assignment the morning of the race. Lanes are numbered from left to right as shown in the image below.

<img src="https://raw.githubusercontent.com/mit-rss/final_challenge2023/main/media/start_area.jpg" width="400"/>

Your car's task is to complete the 200-meter loop around the track as fast as possible, while staying in your assigned lane. Any kind of collision (with another car or with something in Johnson) will be penalized heavily. You should have some kind of safety controller running on your car, but be careful that this doesn't stop your car if there is another car driving next to it on the track! 

### Race Day
On race day, multiple teams will set up on the track at the same time. A TA will give the start signal, at which point the race begins! You must have a member of your team closely follow your car along the track with the controller ready to take manual control at any moment (yes, a great opportunity to exercise). Your car's split will be recorded at the finish line, and TAs will also be stationed at various points along the track recording lane breaches, if they occur (but hopefully no collisions). Each team will have the opportunity to race **three** times, and we will take the best score.

### Tips

Here are some things you may consider in developing your approach:

- How can you reliably segment the lane lines?
- How can you obtain information about the lane lines in the world frame?
- How can you detect if the car has drifted into a neighboring lane?

Please note that Hough Transforms will very likely be useful; helpful resources are [here](https://towardsdatascience.com/lines-detection-with-hough-transform-84020b3b1549) and here(https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html).

## FAQ

### Part A: Final Race

*Do we need to design a safety controller for this challenge?* 
* You should run some kind of safety controller during the challenge, but don't need to spend a lot of time adapting it to the race setting. The easiest way to keep the race collision-free will be for each team to design a robust lane-following solution and remain in-lane. Note: some teams showed solutions in Lab 3 that considered a fixed angle range in front of a car only when deciding when to stop the car. **You should make sure that cars racing alongside yours will not wrongly trigger your safety controller, especially when turning bends in the track!** Consider testing with objects in adjacent lanes.

*Will we be penalized if another car comes into our lane and we crash?*
* No. If you stay in your lane, you will not be considered at fault for any collision. We will give every team the opportunity to record three interference-free lap times on race day.

*Doesn't the car in the outside lane have to travel farther?*
* We will stagger the starting locations so every car travels the same distance. You should be prepared to race in any lane.
