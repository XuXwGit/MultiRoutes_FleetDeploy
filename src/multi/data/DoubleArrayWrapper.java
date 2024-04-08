package multi.data;

import java.util.Arrays;

public class DoubleArrayWrapper {
    private final double[] array;

    public DoubleArrayWrapper(double[] array) {
        this.array = array;
    }

    public double[] getArray() {
        return array;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        DoubleArrayWrapper that = (DoubleArrayWrapper) o;
        return Arrays.equals(array, that.array);
    }

    @Override
    public int hashCode() {
        return Arrays.hashCode(array);
    }
}
