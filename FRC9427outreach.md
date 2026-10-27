改機器速度到RobotContainer裡面，找到控制底盤的那裡

                                        double y;
                                        double x;
                                        double turnLeft;
                                        double turnRight;
                                        if (driverController.isConnected()) {
                                                y = -MathUtil.applyDeadband(driverController.getLeftY() , 0.06);
                                                x = -MathUtil.applyDeadband(driverController.getLeftX(), 0.06);
                                                turnLeft = MathUtil.applyDeadband(
                                                                driverController.getLeftTriggerAxis()這邊乘上一個小數例如:*0.6  ,0.01);//其餘的也是一樣道理
                                                turnRight = MathUtil
                                                                .applyDeadband(
                                                                driverController.getRightTriggerAxis() ,0.01);

                                        } else {

                                                y = -MathUtil.applyDeadband(PS5.getLeftY() , 0.06);
                                                x = -MathUtil.applyDeadband(PS5.getLeftX() , 0.06);
                                                turnLeft = MathUtil.applyDeadband(ps5Trigger(PS5.getL2Axis() ),
                                                                0.01);
                                                turnRight = MathUtil.applyDeadband(ps5Trigger(PS5.getR2Axis() ),
                                                                0.01);
                                        }
