insert into grasp
(grasp_id,scaled_model_id,grasp_pregrasp_joints,grasp_grasp_joints,grasp_energy,grasp_pregrasp_position,grasp_grasp_position,grasp_source_id,grasp_pregrasp_clearance,grasp_cluster_rep,hand_name,grasp_table_clearance)
select 1170000,18744,grasp_pregrasp_joints,grasp_grasp_joints,grasp_energy,grasp_pregrasp_position,grasp_grasp_position,grasp_source_id,grasp_pregrasp_clearance,grasp_cluster_rep,hand_name,grasp_table_clearance
from grasp where grasp_id=1129853;


UPDATE grasp
set grasp_pregrasp_joints='{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}',
grasp_grasp_joints='{0,0,0,50,20,20,0,50,20,20,0,50,20,20,0,0,50,20,20,30,0,0,30,80}',
grasp_cluster_rep=true
where grasp_id=1170000;

UPDATE grasp
set grasp_pregrasp_position='{0.0,0.0,0.281533,0.7071067811865476,0.7071067811865476,0.0,0.0}',

grasp_grasp_position='{0.0,0.0,0.213018,0.7071067811865476,0.7071067811865476,0.0,0.0}'
where grasp_id=1170000

