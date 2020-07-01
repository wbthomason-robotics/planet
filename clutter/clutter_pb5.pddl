(define (problem pb7)
 (:domain clutter)
 (:objects
  table1 table2 table3 table4 -surface
  lgrippertoolframe rgrippertoolframe -manipulator
  bs1 bs2 bs3 gs1 gs2 -objs )
 (:init
  (on-surface bs1 table1) (on-surface bs2 table1) (on-surface bs3 table1)
  (on-surface gs1 table1) (on-surface gs2 table1)
  (arm-empty lgrippertoolframe) (arm-empty rgrippertoolframe))
 (:goal (and (on-surface bs1 table3) (on-surface bs2 table3) (on-surface bs3 table3) (on-surface gs1 table4) (on-surface gs2 table4) (arm-empty rgrippertoolframe) (arm-empty lgrippertoolframe))))
