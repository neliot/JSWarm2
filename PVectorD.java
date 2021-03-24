/************************************************
* PVectorD Class - double based implementation of 
* PVector
*************************************************
* See history.txt
*/

class PVectorD {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    static PRNG rand = new PRNG();

    PVectorD() {
    }

    PVectorD(double xx, double yy) {
        this.x=xx;
        this.y=yy;
        this.z=0;
    }

    PVectorD(double xx, double yy, double zz) {
        this.x=xx;
        this.y=yy;
        this.z=zz;
    }

    static public PVectorD add(PVectorD v1, PVectorD v2) {
        PVectorD result = new PVectorD((v1.x + v2.x),(v1.y + v2.y),(v1.z + v2.z));
        return result;
    }

    static public PVectorD add(PVectorD v1, PVectorD v2, PVectorD target) {
        if (target == null) {
            target = new PVectorD(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
        } else {
            target.set(v1.x + v2.x, v1.y + v2.y, v1.z + v2.z);
        }
        return target;
    }
    
    static public PVectorD sub(PVectorD v1, PVectorD v2) {
        PVectorD result = new PVectorD((v1.x - v2.x),(v1.y - v2.y),(v1.z - v2.z));
        return result;
    }

    static public PVectorD sub(PVectorD v1, PVectorD v2, PVectorD target) {
        if (target == null) {
            target = new PVectorD(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
        } else {
            target.set(v1.x - v2.x, v1.y - v2.y, v1.z - v2.z);
        }
        return target;
    }

    static public PVectorD mult(PVectorD v, double n, PVectorD target) {
        if (target == null) {
            target = new PVectorD(v.x*n, v.y*n, v.z*n);
        } else {
            target.set(v.x*n, v.y*n, v.z*n);
        }
        return target;
    }

    static public PVectorD div(PVectorD v, double n) {
        v.set(v.x*n, v.y*n, v.z*n);
        return div(v, n, null);
    }

    static public PVectorD div(PVectorD v, double n, PVectorD target) {
        if (target == null) {
        target = new PVectorD(v.x/n, v.y/n, v.z/n);
        } else {
        target.set(v.x/n, v.y/n, v.z/n);
        }
        return target;
    }

    static public double dist(PVectorD v1, PVectorD v2) {
        double dx = v1.x - v2.x;
        double dy = v1.y - v2.y;
        double dz = v1.z - v2.z;
        return Math.sqrt(dx*dx + dy*dy + dz*dz);
    }

    static public double dot(PVectorD v1, PVectorD v2) {
        return v1.x*v2.x + v1.y*v2.y + v1.z*v2.z;
    }

    static public PVectorD cross(PVectorD v1, PVectorD v2, PVectorD target) {
        double crossX = v1.y * v2.z - v2.y * v1.z;
        double crossY = v1.z * v2.x - v2.z * v1.x;
        double crossZ = v1.x * v2.y - v2.x * v1.y;

        if (target == null) {
            target = new PVectorD(crossX, crossY, crossZ);
        } else {
            target.set(crossX, crossY, crossZ);
        }
        return target;
    }

    static public PVectorD random2D() {
        PVectorD result = new PVectorD(0.5-rand.nextDouble(),0.5-rand.nextDouble());
        result.setMag(1);
        return result;
    }

    static public double angleBetween(PVectorD v1, PVectorD v2) {

        if (v1.x == 0 && v1.y == 0 && v1.z == 0 ) return 0.0d;
        if (v2.x == 0 && v2.y == 0 && v2.z == 0 ) return 0.0d;

        double dot = v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
        double v1mag = Math.sqrt(v1.x * v1.x + v1.y * v1.y + v1.z * v1.z);
        double v2mag = Math.sqrt(v2.x * v2.x + v2.y * v2.y + v2.z * v2.z);
        // This should be a number between -1 and 1, since it's "normalized"
        double amt = dot / (v1mag * v2mag);
        // But if it's not due to rounding error, then we need to fix it
        // http://code.google.com/p/processing/issues/detail?id=340
        // Otherwise if outside the range, acos() will return NaN
        // http://www.cppreference.com/wiki/c/math/acos
        if (amt <= -1) {
            return Math.PI;
        } else if (amt >= 1) {
        // http://code.google.com/p/processing/issues/detail?id=435
            return 0;
        }
        return Math.acos(amt);
    }

    public PVectorD set (double xx, double yy) {
        this.x=xx;
        this.y=yy;
        return this;
    }

    public PVectorD set (double xx, double yy, double zz) {
        this.x=xx;
        this.y=yy;
        this.z=zz;
        return this;
    }

    public PVectorD set (PVectorD p) {
        this.x=p.x;
        this.y=p.y;
        this.z=p.z;
        return this;
    }

    public String toString() {
        return "[ " + this.x + ", " + this.y + ", " + this.z + " ]";
    }

    public PVectorD copy() {
        return new PVectorD(this.x, this.y, this.z);
    }

    public PVectorD get() {
        return this.copy();
    }

    public double mag() {
        return Math.sqrt(this.x*this.x + this.y*this.y + this.z*this.z);
    }

    public double magSq() {
        return (this.x*this.x + this.y*this.y + this.z*this.z);
    }

    public PVectorD add(PVectorD v) {
        this.x += v.x;
        this.y += v.y;
        this.z += v.z;
        return this;
    }

    public PVectorD add(double x, double y) {
        this.x += x;
        this.y += y;
        return this;
    }

    public PVectorD add(double x, double y, double z) {
        this.x += x;
        this.y += y;
        this.z += z;
        return this;
    }

    public PVectorD sub(PVectorD v) {
        this.x -= v.x;
        this.y -= v.y;
        this.z -= v.z;
        return this;
    }

    public PVectorD sub(double x, double y) {
        this.x -= x;
        this.y -= y;
        return this;
    }

    public PVectorD mult(double n) {
        this.x *= n;
        this.y *= n;
        this.z *= n;
        return this;
    }

    public PVectorD div(double n) {
        this.x /= n;
        this.y /= n;
        this.z /= n;
        return this;
    }

    public double dist(PVectorD v) {
        double dx = this.x - v.x;
        double dy = this.y - v.y;
        double dz = this.z - v.z;
        return Math.sqrt(dx*dx + dy*dy + dz*dz);
    }

    public double dot(PVectorD v) {
        return x*v.x + y*v.y + z*v.z;
    }

    public double dot(double x, double y, double z) {
        return this.x*x + this.y*y + this.z*z;
    }

    public PVectorD cross(PVectorD v) {
        return cross(v, null);
    }

    public PVectorD cross(PVectorD v, PVectorD target) {
        double crossX = y * v.z - v.y * z;
        double crossY = z * v.x - v.z * x;
        double crossZ = x * v.y - v.x * y;

        if (target == null) {
            target = new PVectorD(crossX, crossY, crossZ);
        } else {
            target.set(crossX, crossY, crossZ);
        }
        return target;
    }

    public PVectorD normalize() {
        double m = mag();
        if (m != 0 && m != 1) {
            div(m);
        }
        return this;
    }

    public PVectorD normalize(PVectorD target) {
        if (target == null) {
            target = new PVectorD();
        }
        double m = mag();
        if (m > 0) {
            target.set(x/m, y/m, z/m);
        } else {
            target.set(x, y, z);
        }
        return target;
    }

    public PVectorD limit(double max) {
        if (magSq() > max*max) {
            this.normalize();
            this.mult(max);
        }
        return this;
    }

    public PVectorD setMag(double len) {
        this.normalize();
        this.mult(len);
        return this;
    }

    public PVectorD setMag(PVectorD target, double len) {
        target = this.normalize(target);
        target.mult(len);
        return target;
    }

    public double heading() {
        double angle = Math.atan2(y, x);
        return angle;
    }

    public PVectorD rotate(double theta) {
        double temp = x;
        x = x*Math.cos(theta) - y*Math.sin(theta);
        y = temp*Math.sin(theta) + y*Math.cos(theta);
        return this;
    }

    public double[] array() {
        double[] array = new double[3];
        array[0] = this.x;
        array[1] = this.y;
        array[2] = this.z;
        return array;
    }

    public boolean equals(Object obj) {
        if (!(obj instanceof PVectorD)) {
            return false;
        }
        final PVectorD p = (PVectorD) obj;
        return x == p.x && y == p.y && z == p.z;
    }

    public int hashCode() {
        int result = 1;
        result = 31 * result + Float.floatToIntBits((float)x);
        result = 31 * result + Float.floatToIntBits((float)y);
        result = 31 * result + Float.floatToIntBits((float)z);
        return result;
    }
}