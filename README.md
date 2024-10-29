# Pedestrian-Analysis
It reads raw data (reference_CES48.csv) of the shoulder, pelvis, knee, and ankle angles viewed from the right and back of a pedestrian, and stores comparative data in a new CSV file.

**<main.m>**
1. select case
   - select the direction view (right or back)
   - load the appropriate data
2. calculation **<calculation.m>**
   - use the Kalman filter to filter data
   - calculate the mean
   - calculate the standard deviation
3. show graph **<show_graph_r.m>**, **<show_graph_b.m>**
   - gather data
   - draw graph
   - save data
   - save graph
