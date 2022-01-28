package org.firstinspires.ftc.teamcode.Drivers;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Control.Robot;

public class _Servo {

    private final String _NAME;
    private final double _MAX;
    private final double _MIN;

    private final Servo _servo;
    private double _position;
    private double _targetPosition;
    private double _incrementPosition;
    private int _direction;
    private double _intervalMS;
    private double _lastUpdateTime;
    private boolean _isBusy;

    _Servo(String name, Servo.Direction direction, double min, double max, double start) {
        _NAME = name;
        _MAX = max;
        _MIN = min;
        _servo = Robot.hardwareMap.servo.get(_NAME);
        _servo.setDirection(direction);
        _setPosition(start);
    }

    public void update() {
        if (_isBusy) {
            if (_direction > 0 ? _position >= _targetPosition : _position <= _targetPosition) {
                _resetForNextRun();
            }
            else if (Robot.runtime.milliseconds() >= _lastUpdateTime + _intervalMS) {
                _setPosition(_position + _incrementPosition);
                _lastUpdateTime = Robot.runtime.milliseconds();

                if (_direction > 0 ? _position >= _targetPosition : _position <= _targetPosition) {
                    _resetForNextRun();
                }
            }
        }
    }

    public void resetForNextRun() {
        _isBusy = false;
    }

    public void setPosition(double position) {
        if (!_isBusy) {
            _setPosition(position);
        }
    }

    public void setSlowPosition(double position, double increment, double intervalMS) {
        if (!_isBusy) {
            _isBusy = true;
            _targetPosition = position;
            _direction = _targetPosition > _position ? 1 : -1;
            _incrementPosition = _direction * Math.abs(increment);
            _intervalMS = intervalMS;
            _lastUpdateTime = Robot.runtime.milliseconds() - _intervalMS;
        }
    }

    public void setSlowPosition(double position, double increment) {
        setSlowPosition(position, increment, 10);
    }

    public String getName() {
        return _NAME;
    }

    public boolean isBusy() {
        return _isBusy;
    }

    private void _resetForNextRun() {
        _isBusy = false;
        _setPosition(_targetPosition);
    }

    private void _setPosition(double position) {
        _position = Math.max(Math.min(position, _MAX - _MIN), 0);
        _servo.setPosition(_position + _MIN);
    }
}