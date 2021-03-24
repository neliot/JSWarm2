/************************************************
* Obstacle Class
*************************************************
* See history.txt
*/
import org.json.JSONException;
import org.json.JSONObject;

class Obstacle {
  static int _nextObsId = 0;
  int _id;
  PVectorD _loc;
  double _Ob = 50;
  
  Obstacle(int i, double x, double y, double z) {
/** 
* Creates an Obstacle
* 
* @param i Agent Id
* @param x location
* @param y location
* @param z location
*/
    this._id = i;
    this._loc = new PVectorD(x,y,z);
  }
  
  Obstacle(int i, double x, double y, double z, double Ob) {
/** 
* Creates a desination
* 
* @param i Agent Id
* @param x location
* @param y location
* @param z location
* @param range repulsion range of Obstacle
*/
    this._id = i;
    this._Ob = Ob;
    this._loc = new PVectorD(x,y,z);
  }

  public JSONObject getJSONProps() {
    JSONObject o = new JSONObject();
    try {
      o.put("id", this._id);
      o.put("ob",_Ob);
    } catch (JSONException e) {
      e.printStackTrace();
    }
    return o;
  }

  public void setPos(double x, double y, double z) {
    this._loc.set(x,y,z);
  }

  public String toString() {
/** 
* Creates a formatted string of destination.
*/
    return(this._id + "," + this._loc.x + "," + this._loc.y + "," + this._loc.z + "," + this._Ob);
  }  
}
