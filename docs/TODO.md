Created by Kriston Rickman

09/01/26

Updated from V1.26

0.5 Saving aircraft data for user so they dont have to repeatedly type in the same information of the same aircraft. Then 
set up the same system for stabilty/trim.

0.75 Add calculation of TW into the calculate_TW function and once no warnings or errors create the graph_TW where it 
switches states to graph the calculated data from user of constraints

1. Calculate stability/trim:
   
   tail volume constraint
   
   tail moment arm
   
   tail horizontal area
   
   vertical stabilaer area
   
   User inputs: AR for vert. and horiz. stabilizer and their taper ratio
   
   stabilizer span
   
   find tip chord
   
   continuation in docs...

   Taper ratio for wing

   Neutral point location

   static margin

   AFT CG limit

   Forward CG limit

   CG range

   Performance Verification

   Limit load

   Wing bending moment

   Gust loads

   Landing gear vertical loads

   

1.5  Based on all data find Thrust to weight ratio. Change formulas and remove assumptions as much as possible for most 

accuracy.



2. I need to add user the options to choose specifically what they want to calculate:

  Easy method: Uses wetted aspect ratio and much assumptions to quickly get a rough estimate

  Checks an existing paper design of aircraft
  
  Compare saved designs

  custom designs based on existing aircrafts
  
  based on the point they chose, it goes back and prints every piece of data that can be used

  user do a range of values to explore different aircrafts that are a variety but still fits the requirements

  Based on most data find HP that fit requirements through using a range of values

  Based on all data find HP

  Based on most data find wing area that fit requirements through using a range of values

  Based on all data find wing area
  
  Based on most data find span that fit requirements through using a range of values

  Based on all data find span

  Based on most data find mass that fit requirements through using a range of values

  Based on all data find mass

  Based on most data find Thrust to weight ratio that fit requirements through using a range of values

  Based on most data find Wing load that fit requirements through using a range of values

  Based on all data find Wing load

  Based on most data find aspect ratio that fit requirements through using a range of values

  Based on all data find aspect ratio


3. Validate information so impossible designs do not process and creates a final design Report that the FAA:

   Basic Geometry
   
   Weight Data
   
   Propulsion
   
   Aerodynamics
   
   Performance
   
   Stability
   
   Structural checks
   
   Constraint result
   
   Verification such as specific things that passed or failed on limits
   
   Assumptions/Warnings
   
