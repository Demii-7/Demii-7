
# PlanAhead Launch Script for Post-Synthesis floorplanning, created by Project Navigator

create_project -name Processor -dir "D:/Task 4 + Bonus/Process_MPU/Process_MPU/Processor_template/Processor_template/planAhead_run_1" -part xc3s100ecp132-4
set_property design_mode GateLvl [get_property srcset [current_run -impl]]
set_property edif_top_file "D:/Task 4 + Bonus/Process_MPU/Process_MPU/Processor_template/Processor_template/top_processor_FPGA.ngc" [ get_property srcset [ current_run ] ]
add_files -norecurse { {D:/Task 4 + Bonus/Process_MPU/Process_MPU/Processor_template/Processor_template} }
set_property target_constrs_file "top_processor_FPGA.ucf" [current_fileset -constrset]
add_files [list {top_processor_FPGA.ucf}] -fileset [get_property constrset [current_run]]
link_design
