# Reflection

---

## Investigation
* Started from equtation: ``` steer = -cte*Kp - diff_ctr*Kd - int_cte*Ki - diff_steer*Ks - speed*Ke ```
* Finaly use only two parameters: ``` steer = -cte*Kp - diff_ctr*Kd ```: there is no bias for Ki, and dependency fron speed and previus steering is not linear

## Effect
* P: Sharp turns and then twist
* I: No need to bias 
* D: Smooth Turns

## Final hyperparameters
Combination of twiddle + manual tuning:
* break twiddle iteration if current error > best error
* skipped already tuned parameters: ``` if (par == 1) par++; ```
* final values: ``` {0.3, 0.000, 3.5} ```
