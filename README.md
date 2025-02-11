# Reef Naming Diagram
Used in **auto align** and naming **autons**


<img width="262" alt="Screenshot 2025-02-07 at 8 15 35 PM" src="https://github.com/user-attachments/assets/2f59b383-8e66-4062-a129-31e69f7fe6b3" />


# Auton 
Every auton name consists of the path names separated by an exclamation point


### Pose names: 
- A-L are reef poses
- LS and RS are Left Source (from alliance wall)  and Right Source respectively
- P is processor


## Auton naming example
**Auton:** 1! 1J! JTS!

**Explanation:**

**1!** the first path leaves the first starting position (left-most from alliance wall)


**1J!** the second path goes from the first path to reef J


**JtLS!** the third path goes from reef J to the Left Source 

**results in:** 

<img width="400" src ="https://github.com/user-attachments/assets/a762738d-5ce7-4dd8-92ae-60dabd413042"/>


## Naming explanation


The **first path** always leaves the starting zone. The starting positions are labeled 1-3 starting from the left-most position from the alliance wall. Ends with velocity of 1 m/s.


<img width="100" alt="Screenshot 2025-02-07 at 8 34 27 PM" src="https://github.com/user-attachments/assets/df266e97-971f-464a-9452-caf26d9d7718" />
<img width= "250" src = "https://github.com/user-attachments/assets/d633ae7e-2277-4dba-a05a-a0a45abb18f7"/> (path 1!) 


The **second path** starts from the end of the first path and goes to the next position listed. These end with a velocity of 0. 


<img width = "250" src ="https://github.com/user-attachments/assets/c1163390-53f9-41c9-a466-11bb1e2a7194"/> (path 1J!)



The **following paths** start from the first position listed and end at the last. The positions are seperated with a "t". For example LtRS is pose L to Right Source. 


<img width = "250" src="https://github.com/user-attachments/assets/21595538-5714-4bbe-8fd0-a374f7cc71d1" /> (path JtLS!)


