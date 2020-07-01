(define (problem pb7)
 (:domain clutter)
 (:objects
  table1 table2 table3 table4 -surface
  lgrippertoolframe rgrippertoolframe -manipulator
  bs1 -objs )
 (:init
  (on-surface bs1 table1) (arm-empty lgrippertoolframe) (arm-empty rgrippertoolframe))
 (:goal (and (on-surface bs1 table3) (arm-empty rgrippertoolframe) (arm-empty lgrippertoolframe))))
