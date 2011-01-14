# Containts all the information from one run of the tabletop detection node

# The information for the plane that has been detected
Table table

# The raw clusters detected in the scan 
sensor_msgs/PointCloud[] clusters

# The list of models that have been detected
household_objects_database_msgs/DatabaseModelPose[] models

# For each cluster, the index of the model that was fit to that cluster
# or -1 if no fit was successful for that cluster
# keep in mind that multiple raw clusters can correspond to a single fit
int32[] cluster_model_indices

# Whether the detection has succeeded or failed
int32 NO_CLOUD_RECEIVED = 1
int32 NO_TABLE = 2
int32 OTHER_ERROR = 3
int32 SUCCESS = 4
int32 result