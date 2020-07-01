(define (problem pb7)
 (:domain clutter)
 (:objects
  table1 table2 table3 table4 -surface
  lgrippertoolframe rgrippertoolframe -manipulator
  bs1 gs1 -objs
  l_upper_arm_link l_forearm_link r_upper_arm_link r_forearm_link -link)
 (:init
  (on-surface bs1 table1) (on-surface gs1 table1) (arm-empty lgrippertoolframe) (arm-empty rgrippertoolframe))
 (:goal (and (on-surface bs1 table3) (on-surface gs1 table4) (arm-empty rgrippertoolframe) (arm-empty lgrippertoolframe))))
