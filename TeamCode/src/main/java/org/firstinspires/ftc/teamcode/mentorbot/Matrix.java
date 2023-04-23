package org.firstinspires.ftc.teamcode.mentorbot;

public class Matrix {

    int rows;
    int columns;
    double[][] matrixArray;

    public Matrix(int rows, int columns) {
        this.rows = rows;
        this.columns = columns;
        matrixArray = new double[this.rows][this.columns];
    }

    public Matrix initRow(int row, double[] rowArray) {
        if (columns >= 0) System.arraycopy(rowArray, 0, matrixArray[row], 0, columns);

        return this;
    }

    public void setRow(int row, double[] rowArray) {
        if (columns >= 0) System.arraycopy(rowArray, 0, matrixArray[row], 0, columns);
    }

    public static Matrix add(Matrix a, Matrix b) {
        if (a.rows != b.rows || a.columns != b.columns) {
            throw new IllegalArgumentException("Matrix dimensions do not match!");
        }

        Matrix output = new Matrix(a.rows, a.columns);

        for (int i = 0; i < a.rows; i++) {
            for (int j = 0; j < a.columns; j++) {
                output.matrixArray[i][j] = a.matrixArray[i][j] + b.matrixArray[i][j];
            }
        }

        return output;
    }

    public static Matrix multiply(Matrix a, Matrix b) {
        if (a.columns != b.rows) {
            throw new IllegalArgumentException("A's columns do not match B's rows!");
        }

        Matrix output = new Matrix(a.rows, b.columns);

        for (int i = 0; i < a.rows; i++) {
            for (int j = 0; j < b.columns; j++) {
                for (int k = 0; k < a.columns; k++) {
                    output.matrixArray[i][j] += a.matrixArray[i][k] * b.matrixArray[k][j];
                }
            }
        }

        return output;
    }
}
