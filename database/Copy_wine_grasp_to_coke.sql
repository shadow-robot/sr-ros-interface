insert into grasp
(grasp_id,scaled_model_id,grasp_pregrasp_joints,grasp_grasp_joints,grasp_energy,grasp_pregrasp_position,grasp_grasp_position,grasp_source_id,grasp_pregrasp_clearance,grasp_cluster_rep,hand_name,grasp_table_clearance)
select 1170000,18744,grasp_pregrasp_joints,grasp_grasp_joints,grasp_energy,grasp_pregrasp_position,grasp_grasp_position,grasp_source_id,grasp_pregrasp_clearance,grasp_cluster_rep,hand_name,grasp_table_clearance
from grasp where grasp_id=1129853;


update grasp 
set grasp_pregrasp_joints='{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}',
grasp_grasp_joints='{-25,78,40,0,78,40,0,74,40,0,-25,71,40,-14,70,0,19,27}'
where grasp_id=1170000


UPDATE grasp
set grasp_pregrasp_position='{0.0,0.0,0.241533,0.7071067811865476,0.7071067811865476,0.0,0.0}',
set grasp_grasp_position='{0.0,0.05,0.17,0.7071067811865476,0.7071067811865476,0.0,0.0}'
where grasp_id=1170000

