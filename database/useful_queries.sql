
select * from grasp where hand_name='SHADOW_HAND';

select * from original_model 
where original_model_id in 
(select original_model_id from scaled_model
where scaled_model_id in (select distinct scaled_model_id from grasp
where hand_name='SHADOW_HAND'));

select * from original_model 
where original_model_id in 
( select original_model_id from scaled_model
where scaled_model_id=18744);
