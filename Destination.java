/************************************************
* Destination Class 
*************************************************
* See history.txt
*/
import org.json.JSONObject;
import org.json.JSONException;

class Destination {
  static int _nextDestId = 0;
  int _id;
  PVectorD _loc;
  double _size = 10.0;
  double _mass = 1.0;

  Destination(int i, double x, double y, double z) {
/** 
* Creates a destination
* 
* @param i Destination Id
* @param x location
* @param y location
* @param z location
*/
    this._id = i;
    this._loc = new PVectorD(x,y,z);
  }
  
  Destination(int i, double x, double y, double z, double size, double mass) {
/** 
* Creates a desination
* 
* @param i Agent Id
* @param x location
* @param y location
* @param z location
* @param size diameter
* @param mass mass of destination (Future work maybe?)
*/
    this._id = i;
    this._size = size;
    this._mass = mass;
    this._loc = new PVectorD(x,y,z);
  }

  public void setPos(double x, double y, double z) {
    this._loc.set(x,y,z);
  }

  public JSONObject getJSONProps() {
    JSONObject o = new JSONObject();
    try {
      o.put("id", this._id);
      o.put("size",_size);
      o.put("mass",_mass);
    } catch (JSONException e) {
      e.printStackTrace();
    }
    return o;
  }

  public String toString() {
/** 
* Creates a formatted string of destination.
*/
    return(this._id + "," + this._loc.x + "," + this._loc.y + "," + this._loc.z + "," + this._size + "," + this._mass);
  }  
}
