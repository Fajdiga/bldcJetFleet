; Boost charge manager
;
; Charges the battery through the motor windings. The fast and safety-critical
; parts are in the firmware (motor/boost_charge.c): the charge lock, the current
; loop and the trips. This script decides how much current to ask for, and
; stops asking as soon as anything looks wrong.

; ---- Settings ----
; Stored in the lisp eeprom. The order here is the order the UI uses.

(def eeprom-base 200)
(def eeprom-magic 0x42C1)

(def set-names '(
        i-max        ; Maximum input current [A]
        cell-target  ; Highest cell voltage to charge to [V]
        i-term       ; Stop when the input current falls below this [A]
        v-sup-min    ; Supply voltage window [V], 0 = from the voltage seen at the start
        v-sup-max
        v-bat-max    ; Pack over-voltage trip [V], 0 = cells * (cell-target + 0.07)
        f-sw         ; Switching frequency [kHz], 0 = keep the motor setting
        p-sup-max    ; Supply power limit [W], 0 = off
        i-bat-max    ; Battery charge current limit [A], 0 = off
        ah-max       ; Charge limit [Ah], 0 = off
        h-max        ; Time limit [h], 0 = off
        t-cell-min   ; Cell temperature window [degC]
        t-cell-max
        t-mot-max    ; Motor temperature limit [degC], derates over the 10 degC below
        t-fet-max    ; FET temperature limit [degC], derates over the 10 degC below
        i-slew       ; Current ramp rate [A/s]
        i-align      ; Rotor alignment current [A]
        t-align      ; Rotor alignment hold time [s]
        kp           ; Current loop gains, 0 = from the motor configuration
        ki
        phase        ; Motor phase the supply is connected to, 1 to 3
        i-trip       ; Over-current trip [A], 0 = 1.5 * i-max + 5 + worst case ripple
        auto-start   ; 1 = start as soon as the charger is plugged in
        v-sup-k      ; Supply voltage reading correction while switching, 0 = board default
        v-bat-k      ; Battery voltage reading correction while switching, 0 = board default
))

(def settings (list
        5.0 4.2 0.5 0.0 0.0 0.0 0.0 0.0 0.0 0.0 0.0
        0.0 45.0 80.0 80.0 2.0 2.0 1.0 0.0 0.0 1.0 0.0 0.0 0.0 0.0))

; ---- State ----

(def st-idle 0)      ; Motor mode allowed
(def st-wait 1)      ; Charge lock set, waiting for the supply
(def st-precheck 2)
(def st-align 3)
(def st-ramp 4)
(def st-cc 5)
(def st-cv 6)
(def st-done 7)
(def st-fault 8)
(def st-stopped 9)   ; Stopped by the user, lock stays set
(def st-manual 10)   ; Bench test: fixed current, no cell checks
(def st-ready 11)    ; Supply found, waiting for the start button

(def state st-idle)
(def state-t (systime))
(def msg "")
(def i-set 0.0)
(def i-manual 0.0)
(def ah 0.0)
(def wh 0.0)
(def chg-secs 0.0)
(def sup-seen false)
(def v-sup-last 0.0)
(def v-steady-cnt 0)
(def term-cnt 0)
(def go-req false)   ; Start was pressed
(def use-bms true)   ; false = no BMS on the bus, charge on the pack voltage
(def v-bat-now 0.0)
(def v-sup-ref 0.0)  ; Supply voltage the automatic lower limit follows
(def f-zv-orig (conf-get 'foc-f-zv))

; The functions and tables are stored in flash, the heap is small
@const-start

(defun set-index (name) {
        (var i 0)
        (var res -1)
        (loopforeach it set-names {
                (if (eq it name) (setq res i))
                (setq i (+ i 1))
        })
        res
})

(defun s (name) (ix settings (set-index name)))

(defun load-settings () {
        (var magic (eeprom-read-i eeprom-base))
        (if (and magic (= magic eeprom-magic))
            (looprange i 0 (length settings) {
                    (var v (eeprom-read-f (+ eeprom-base 1 i)))
                    (if v (setix settings i v))
        }))
})

(defun save-settings () {
        (looprange i 0 (length settings)
            (eeprom-store-f (+ eeprom-base 1 i) (ix settings i)))
        (eeprom-store-i eeprom-base eeprom-magic)
})

; Same order as boost_trip_t in motor/boost_charge.h
(def trip-names '(
        "none"
        "pack over-voltage"
        "over-current"
        "current flowing back into the supply"
        "supply too close to the battery voltage"
        "supply voltage too low or unplugged"
        "supply over-voltage"
        "setpoint timeout"
        "VESC fault"
        "no stored current offsets, boot once without the charger"
        "bad configuration"
        "no current: charger not connected, or its voltage is too low for this pack"
        "charger voltage too low for this battery (needs about 12 % of the battery voltage + 2 V)"
))

(defun min2 (a b) (if (< a b) a b))
(defun max2 (a b) (if (> a b) a b))
(defun clamp (x lo hi) (if (< x lo) lo (if (> x hi) hi x)))

(defun go (new-state) {
        (setq state new-state)
        (setq state-t (systime))
})

(defun fault (text) {
        (boost-stop)
        (setq i-set 0.0)
        (setq msg text)
        (go st-fault)
})

(defun cells ()
    (max2 1 (if (and use-bms (> (get-bms-val 'bms-cell-num) 0))
            (get-bms-val 'bms-cell-num)
            (conf-get 'si-battery-cells)
)))

; Without a BMS the average cell voltage is all there is
(defun cell-max () (if use-bms (get-bms-val 'bms-v-cell-max) (/ v-bat-now (cells))))
(defun cell-min () (if use-bms (get-bms-val 'bms-v-cell-min) (/ v-bat-now (cells))))

; ESC battery reading / BMS pack voltage. The BMS is the accurate one, so
; voltage limits are converted to what the ESC will read. 1.0 without a BMS.
(defun v-esc-scale () {
        (var v-bms (get-bms-val 'bms-v-tot))
        (if (and use-bms (> v-bms 10.0) (> v-bat-now 10.0))
            (clamp (/ v-bat-now v-bms) 0.95 1.05)
            1.0)
})

; Worst case ripple of the winding current, at a duty of 0.5. The current can
; start that far off its average (see the trip below).
(defun ripple-max () {
        (var f-sw (if (> (s 'f-sw) 0.0) (* 1000.0 (s 'f-sw)) (/ (conf-get 'foc-f-zv) 2.0)))
        (var l-tot (* 1.5e-6 (conf-get 'foc-motor-l)))
        (if (> l-tot 0.0) (/ v-bat-now (* 4.0 l-tot f-sw)) 0.0)
})

(defun apply-config () {
        (var i-max (s 'i-max))
        (boost-config
            i-max
            (s 'i-slew)
            (if (> (s 'i-trip) 0.0) (s 'i-trip) (+ (* 1.5 i-max) 5.0 (ripple-max)))
            ; The trip runs on the ESC voltage reading, the limit is in real volts
            (* (v-esc-scale) (if (> (s 'v-bat-max) 0.0) (s 'v-bat-max) (* (cells) (+ (s 'cell-target) 0.07))))
            (sup-floor)
            ; 0 = the firmware trips 10 % above the supply voltage and follows it up
            (s 'v-sup-max)
            (s 'kp)
            (s 'ki)
            (- (to-i (s 'phase)) 1)
            (s 'v-sup-k)
            (s 'v-bat-k)
        )
})

; The switching frequency is half of the zero vector frequency
(defun set-f-zv (f-zv)
    (if (> (abs (- (conf-get 'foc-f-zv) f-zv)) 1.0)
        (conf-set 'foc-f-zv f-zv)
))

(defun bms-fresh () (< (get-bms-val 'bms-msg-age) 2.0))

; Why charging must not run right now, or nil if all is fine
(defun cell-problem () {
        (var v-max (cell-max))
        (var v-min (cell-min))
        ; ponytail: only the warmest cell is checked against both limits. Use
        ; the per-sensor temperatures if the pack can be cold on one side only.
        (var temp (get-bms-val 'bms-temp-cell-max))
        (cond
            ((and use-bms (not (bms-fresh))) "cell data is missing or older than 2 s")
            ((> v-max (+ (s 'cell-target) 0.05))
                (if use-bms "cell over-voltage"
                    "pack voltage is above full. Check the cell count in Motor Settings > Additional Info"))
            ((< v-min 2.0)
                (if use-bms "cell voltage below 2.0 V"
                    "pack voltage is too low. Check the cell count in Motor Settings > Additional Info"))
            ((and use-bms (< temp (s 't-cell-min))) "cells too cold to charge")
            ((and use-bms (> temp (s 't-cell-max))) "cells too hot to charge")
            (t nil)
        )
})

(defun derate (temp t-max) (clamp (/ (- t-max temp) 10.0) 0.0 1.0))

; The lowest of the motor limit, the supply power limit and the battery
; charge limit converted to the supply side, then derated.
(defun current-limit (v-sup v-bat) {
        (var lim (s 'i-max))
        (if (> (s 'p-sup-max) 0.0)
            (setq lim (min2 lim (/ (s 'p-sup-max) (max2 v-sup 1.0)))))
        (if (> (s 'i-bat-max) 0.0)
            (setq lim (min2 lim (/ (* (s 'i-bat-max) v-bat) (max2 v-sup 1.0)))))
        (setq lim (* lim (derate (get-temp-mot) (s 't-mot-max))))
        (setq lim (* lim (derate (get-temp-fet) (s 't-fet-max))))
        ; Back off when the supply sags towards its lower limit: the set one, or 80 %
        ; of the supply voltage, which follows a supply that is turned up or down
        (var lo (if (> (s 'v-sup-min) 0.0) (s 'v-sup-min) (* 0.8 v-sup-ref)))
        (setq lim (* lim (clamp (/ (- v-sup lo) (max2 (* 0.05 lo) 0.1)) 0.0 1.0)))
        lim
})

; A supply is on the phase. Without a set window the firmware charger detection
; threshold is used: the phases float to around 12 V through leakage.
(defun sup-floor () (if (> (s 'v-sup-min) 0.0) (s 'v-sup-min) (max2 15.0 (* 0.25 v-bat-now))))

(defun sup-present (v-sup)
    (and (> v-sup (sup-floor))
        (or (<= (s 'v-sup-max) 0.0) (<= v-sup (s 'v-sup-max)))
))

(defun start-charge (v-sup) {
        (setq go-req false)
        ; A BMS that was heard in the last 30 s must stay alive for the whole charge
        (setq use-bms (< (get-bms-val 'bms-msg-age) 30.0))
        (setq v-sup-ref v-sup)
        ; Without cell checks the pack limit is the only stop
        (if (and (> i-manual 0.0) (<= (s 'v-bat-max) 0.0))
            (fault "set the pack voltage limit before a bench test")
            {
                (if (> (s 'f-sw) 0.0) (set-f-zv (* 2000.0 (s 'f-sw))))
                (apply-config)
                (setq ah 0.0)
                (setq wh 0.0)
                (setq chg-secs 0.0)
                (go (if (> i-manual 0.0) st-manual st-precheck))
        })
})

(defun charging () (or (= state st-align) (= state st-ramp) (= state st-cc)
        (= state st-cv) (= state st-manual)))

(defun tick (dt) {
        (var bs (boost-status))
        (var lock (= (ix bs 0) 1))
        (var active (= (ix bs 1) 1))
        (var trip (ix bs 2))
        (var v-sup (ix bs 3))
        (var i-in (ix bs 4))
        (var v-bat (ix bs 5))
        (var i-ref (ix bs 7))
        (var v-low-time (ix bs 8))
        (var in-window (sup-present v-sup))
        (setq v-bat-now v-bat)

        (if in-window (setq sup-seen true))

        ; The lock can be set by the firmware on its own: charger found at
        ; boot, or plugged in while the outputs were off.
        (if (and lock (= state st-idle)) {
                (setq sup-seen false)
                (setq msg "")
                (go st-wait)
        })

        (if (and (not lock) (!= state st-idle)) {
                (setq go-req false)
                (go st-idle)
        })

        ; Unplugged after the supply was seen: give the motor back
        (if (and lock sup-seen (not active) (>= v-low-time 2.0))
            (if (boost-lock 0) {
                    (setq sup-seen false)
                    (set-f-zv f-zv-orig)
                    (go st-idle)
        }))

        ; A trip in the firmware always ends in the fault state
        (if (and (!= trip 0) (!= state st-fault) (!= state st-idle))
            (fault (ix trip-names trip)))

        (if (charging) {
                ; Up at once, down within about 10 s (a sag under load recovers sooner)
                (setq v-sup-ref (if (> v-sup v-sup-ref) v-sup
                        (+ v-sup-ref (* (- v-sup v-sup-ref) (min2 (/ dt 10.0) 1.0)))))
                (setq chg-secs (+ chg-secs dt))
                (setq wh (+ wh (/ (* v-sup i-in dt) 3600.0)))
                ; Battery side estimate, losses not included
                (setq ah (+ ah (/ (* i-in (/ v-sup (max2 v-bat 1.0)) dt) 3600.0)))

                (if (and (> (s 'ah-max) 0.0) (> ah (s 'ah-max)))
                    (fault "amp-hour limit reached"))
                (if (and (> (s 'h-max) 0.0) (> chg-secs (* 3600.0 (s 'h-max))))
                    (fault "time limit reached"))
                (if (and (!= state st-manual) (cell-problem))
                    (fault (cell-problem)))
        })

        (cond
            ((= state st-wait) {
                    (setq i-set 0.0)
                    (if (and in-window (< (abs (- v-sup v-sup-last)) 1.0))
                        (setq v-steady-cnt (+ v-steady-cnt 1))
                        (setq v-steady-cnt 0))
                    (setq v-sup-last v-sup)

                    (if (>= v-steady-cnt 10) {
                            (setq v-steady-cnt 0)
                            (if (or go-req (> i-manual 0.0) (> (s 'auto-start) 0.5))
                                (start-charge v-sup)
                                (go st-ready))
                    })
            })

            ((= state st-ready) (if (not in-window) (go st-wait)))

            ((= state st-precheck)
                (if (cell-problem)
                    (fault (cell-problem))
                    (if (>= (cell-max) (s 'cell-target))
                        {
                            (setq msg "Battery is already full")
                            (go st-done)
                        }
                        (go st-align)
            )))

            ; Both boost legs together pull the rotor to the supply phase.
            ; The firmware ramps the current, then it is held while the rotor settles.
            ((= state st-align) {
                    (setq i-set (min2 (s 'i-align) (current-limit v-sup v-bat)))
                    (if (< i-ref (* 0.95 i-set)) (setq state-t (systime)))
                    (if (> (secs-since state-t) (s 't-align)) (go st-ramp))
            })

            ((= state st-ramp) {
                    (setq i-set (current-limit v-sup v-bat))
                    (if (>= i-ref (* 0.98 i-set)) (go st-cc))
                    (if (>= (cell-max) (s 'cell-target)) (go st-cv))
            })

            ((= state st-cc) {
                    (setq i-set (current-limit v-sup v-bat))
                    (if (>= (cell-max) (s 'cell-target)) {
                            (setq term-cnt 0)
                            (go st-cv)
                    })
            })

            ; Hold the highest cell at the target: 0.5 A/s per 10 mV of error
            ((= state st-cv) {
                    (var err (- (cell-max) (s 'cell-target)))
                    (setq i-set (clamp (- i-set (* err 50.0 dt)) 0.0 (current-limit v-sup v-bat)))
                    (if (< i-set (s 'i-term))
                        (setq term-cnt (+ term-cnt 1))
                        (setq term-cnt 0))
                    (if (> term-cnt 50) {
                            (setq i-set 0.0)
                            (setq msg "Charge complete")
                            (go st-done)
                    })
            })

            ((= state st-manual)
                (setq i-set (min2 i-manual (current-limit v-sup v-bat))))

            (t (setq i-set 0.0))
        )

        ; This is the refresh the firmware needs at least every 500 ms
        (if (and lock (= trip 0))
            (boost-set-current (if (charging) i-set 0.0)))

        bs
})

; ---- UI ----

(defun send-settings () {
        (var str "set")
        (loopforeach it settings
            (setq str (str-merge str (str-from-n it " %.4f"))))
        (send-data str 6)
})

(defun send-status (bs)
    (send-data (str-merge
            "st"
            (str-from-n state " %d")
            (str-from-n (ix bs 0) " %d")
            (str-from-n (ix bs 2) " %d")
            (str-from-n (ix bs 3) " %.2f")                         ; Supply voltage
            (str-from-n (ix bs 4) " %.2f")                         ; Input current
            (str-from-n (ix bs 5) " %.2f")                         ; Pack voltage
            (str-from-n i-set " %.2f")
            (str-from-n (cell-min) " %.3f")
            (str-from-n (cell-max) " %.3f")
            (str-from-n (get-bms-val 'bms-temp-cell-max) " %.1f")
            (str-from-n (min2 (get-bms-val 'bms-msg-age) 999.0) " %.1f")
            (str-from-n (get-temp-mot) " %.1f")
            (str-from-n (get-temp-fet) " %.1f")
            (str-from-n ah " %.3f")
            (str-from-n wh " %.1f")
            (str-from-n chg-secs " %.0f")
            (str-from-n (ix bs 6) " %.4f")                         ; Duty
            (str-from-n (ix bs 9) " %.1f")                         ; Phase voltages
            (str-from-n (ix bs 10) " %.1f")
            (str-from-n (ix bs 11) " %.1f")
            (str-from-n (cells) " %d")
            (if use-bms " 1" " 0")
            " " msg
) 6))

; The functions below are called from the QML page

; The page asks for the status. A reply goes back to whoever asked, while data
; that is pushed ends up at the last sender, which can be a CAN device.
(defun ui-poll () (send-status (boost-status)))

(defun ui-lock (on)
    (if (= on 1)
        (if (boost-lock 1)
            (setq msg "")
            (setq msg "Charge mode refused: the motor must be stopped and set to FOC"))
        (if (boost-lock 0)
            {
                (set-f-zv f-zv-orig)
                (setq msg "")
            }
            (setq msg "Unplug the charger first"))
))

(defun ui-start () {
        (setq i-manual 0.0)
        (boost-clear-trip)
        (setq msg "")
        (if (!= state st-idle) {
                (setq go-req true)
                (go st-wait)
        })
})

(defun ui-manual (current) {
        (ui-start)
        (setq i-manual current)
})

(defun ui-stop () {
        (boost-stop)
        (setq i-manual 0.0)
        (setq go-req false)
        (if (!= state st-idle) {
                (setq msg "Stopped")
                (go st-stopped)
        })
})

(defun ui-set (index value)
    (if (and (>= index 0) (< index (length settings)) (not (charging)))
        (setix settings index (to-float value))
))

(defun ui-save () {
        (save-settings)
        (send-settings)
        (send-data "msg Settings saved" 6)
})

(defun event-handler ()
    (loopwhile t
        (recv
            ((event-data-rx . (? data)) (trap (eval (read data))))
            (_ nil)
)))

@const-end

(load-settings)

; Charger detection is off in the firmware until this package asks for it. The
; marker is what the firmware finds at the next boot, before this script runs.
; Same address and value as BOOST_EEPROM_ARM_* in motor/boost_charge.h.
(let ((mark (eeprom-read-i (- eeprom-base 1))))
    (if (not (and mark (= mark eeprom-magic)))
        (eeprom-store-i (- eeprom-base 1) eeprom-magic)))
(boost-arm)
(event-register-handler (spawn event-handler))
(event-enable 'event-data-rx)

(def tick-t (systime))

(loopwhile t {
        (var dt (secs-since tick-t))
        (setq tick-t (systime))
        ; An error (such as a full heap for a moment) must not end this loop.
        ; A tick that fails does not refresh the setpoint, so the firmware stops.
        (trap (tick dt))
        (sleep 0.1)
})
