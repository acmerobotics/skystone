package com.acmerobotics.practice;

import java.util.Vector;

public class Vector2d {

    private double x, y;

    public Vector2d (double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d (double theta) {
        this.x = Math.cos(theta);
        this.y = Math.sin(theta);
    }

    public Vector2d unit() {
        double norm = this.norm();
        return new Vector2d(this.x / norm, this.y / norm);
    }

    public Vector2d plus (Vector2d other) {
        return new Vector2d(this.x + other.x, this.y + other.y);
    }

    public Vector2d minus (Vector2d other) {
        return new Vector2d(this.x - other.x, this.y - other.y);
    }

    public double dot (Vector2d other) {
        return this.x * other.x + this.y * other.y;
    }

    public double norm () {
        return Math.hypot(this.x, this.y);
    }

    public Vector2d times (double scalar) {
        return new Vector2d(this.x * scalar, this.y * scalar);
    }

    public double x () {
        return x;
    }

    public double y () {
        return y;
    }

}
