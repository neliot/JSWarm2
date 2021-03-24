/************************************************
* Psystem Class - abstract class for 
*                 implementation of models
*************************************************
* See history.txt
*/
import java.io.IOException;
import java.io.PrintWriter;
import java.io.File;
import java.io.FileInputStream;
import java.nio.file.Path;
import java.nio.file.Files;
import java.time.LocalDate;
import java.time.LocalTime;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;
  
import java.util.ArrayList; 

class PSystem {
  java.util.Properties modelProperties = new java.util.Properties();
  ArrayList<Particle> S = new ArrayList<Particle>();
  ArrayList<Destination> D = new ArrayList<Destination>();
  ArrayList<Obstacle> O = new ArrayList<Obstacle>();
  double _kc, _kr, _kd, _ko, _kg, _Cb, _Rb, _Ob, _pr, _pc, _speed;
  boolean _perimCoord = false;
  boolean _particleOptimise = false;
  boolean _logMin = true;

  String _model = "Linear"; // Text of model type
  String _modelId = "1.0"; // Model number.

  boolean _loggingP = true;
  boolean _loggingN = true;
  Logger plog;
  Logger nClog;
  Logger nRlog;

  PSystem() {
/** 
* Sets up the environment with agents and parameters for the simulation
* 
*/ 
    try {
      modelProperties.load(new FileInputStream("model.properties"));
    } catch(Exception e) {
      System.out.println(e);
      System.exit(-1);
    }
// Default model properties
    if (this._loggingP) {
      this.plog = new Logger("data/csv/"+modelProperties.getProperty("swarmData"));
      this.nClog = new Logger("data/csv/"+modelProperties.getProperty("cohesionData"));
      this.nRlog = new Logger("data/csv/"+modelProperties.getProperty("repulsionData"));
      if (this._logMin) {
        this.plog.dump("STEP,ID,X,Y,PERIM,CX,CY,CMAG,RX,RY,RMAG,IX,IY,IMAG,DX,DY,DMAG,CHANGEX,CHANGEY,CHANGEMAG\n");    
        this.nClog.dump("STEP,PID,PX,PY,PPERIM,NID,NX,NY,NPERIM,COHX,COHY,COHZ,MAG,DIST\n");    
        this.nRlog.dump("STEP,PID,PX,PY,PPERIM,NID,NX,NY,NPERIM,REPX,REPY,REPZ,MAG\n");  
      } else {
        this.plog.dump("STEP,ID,X,Y,Z,RANGE,REPULSE,SIZE,MASS,PERIM,CX,CY,CZ,CMAG,RX,RY,RZ,RMAG,IX,IY,IZ,IMAG,AX,AY,AZ,AMAG,DX,DY,DZ,DMAG,CHANGEX,CHANGEY,CHANGEZ,CHANGEMAG\n");    
        this.nClog.dump("STEP,PID,NID,X,Y,Z,RANGE,REPULSE,SIZE,MASS,PERIM,COHX,COHY,COHZ,MAG,DIST\n");    
        this.nRlog.dump("STEP,PID,NID,X,Y,Z,RANGE,REPULSE,SIZE,MASS,PERIM,REPX,REPY,REPZ,MAG\n");  
      }
    } 
    this.loadSwarm();
    this.init();  
  }

  void init() {};

  // void populate() {
  //   PRNG rand = new PRNG(_seed);
  //   for(int i = 0; i < this._swarmSize; i++) {
  //     try {
  //       // create agent in centred quartile.
  //       S.add(new Particle(Particle._nextParticleId++,_grid/2 - rand.nextInt(_grid),(float)_grid/2 - rand.nextInt(_grid),0.0,this._Cb, this._Rb, this._speed));
  //     } catch (Exception e) {
  //       System.out.println(e);
  //       System.exit(-1);
  //     }
  //   }
  // }

  void update() {
/** 
* Update system - Updates particle positions.
*/
    String pData = "";
    PVectorD change = new PVectorD();
    PVectorD stepChange = new PVectorD();
    PVectorD avoid = new PVectorD();
    PVectorD dir = new PVectorD();
    PVectorD coh = new PVectorD();
    PVectorD rep = new PVectorD();
    PVectorD inter = new PVectorD();
    for(Particle p : S) {      
      avoid.set(0,0,0);
      dir.set(0,0,0);
      change.set(0,0,0); 

      p.nbr(S);

      /* Calculate Cohesion */
      coh = cohesion(p);

      /* Calculate Repulsion */
      rep = repulsion(p);

      /* Calculate Obstacle avoidance */
      if (this.O.size() > 0) {
        avoid = avoidObstacles(p);
      }

      if (D.size() > 0) {
        dir = direction(p);
      }
      change.add(dir);
      change.add(avoid);
      change.add(coh);
      change.add(rep);
      
      inter = PVectorD.add(coh,rep);
      stepChange = change.copy();
      stepChange.setMag(p._topspeed);
      
      if (this._loggingP) {
        if (_logMin) {
          pData += plog._counter + "," + p.logString(_logMin) + "," + coh.x + "," + coh.y + "," + coh.mag() + "," + rep.x + "," + rep.y + "," + rep.mag() + "," + inter.x + "," + inter.y + "," + inter.mag() + "," + dir.x + "," + dir.y + "," + dir.mag() + "," + stepChange.x + "," + stepChange.y + "," + stepChange.mag() + "\n";
        } else {
          pData += plog._counter + "," + p.logString(_logMin) + "," + coh.x + "," + coh.y + "," + coh.z + "," + coh.mag() + "," + rep.x + "," + rep.y + "," +  rep.z + "," + rep.mag() + "," + inter.x + "," + inter.y + "," +  inter.z + "," + inter.mag() + "," + avoid.x + "," + avoid.y + "," + avoid.z + "," + avoid.mag() + "," + dir.x + "," + dir.y + "," + dir.z + "," + dir.mag() + "," + stepChange.x + "," + stepChange.y + "," + stepChange.z + "," + stepChange.mag() + "\n";
        }
      }
      p.setChange(change);
    }
    for(Particle p : S) {
      p.update(this._particleOptimise);
    }
    if (this._loggingP) {
      plog.dump(pData);
      plog.clean();
    }
  }
    
  PVectorD cohesion(Particle p) {
/** 
* cohesion calculation - Calculates the cohesion between each agent and its neigbours.
* 
* @param p The particle that is currently being checked
*/
    PVectorD vcb = new PVectorD(0,0);
    PVectorD v = new PVectorD(0,0);
    double distance = 0.0;
    String nData = "";
    
// GET ALL THE NEIGHBOURS
    for(Particle n : p._nbr) {
      if (p._loc.x == n._loc.x && p._loc.y == n._loc.y) {
        System.out.println("ERROR:" + n._id + ":" + p._id);
        System.exit(-1);
      }
      distance = PVectorD.dist(p._loc,n._loc);
      if (p._isPerim && n._isPerim) {
        v = PVectorD.sub(n._loc,p._loc).mult(this._pc).mult(this._kc);
      } else {
        v = PVectorD.sub(n._loc,p._loc).mult(this._kc);
      }
      vcb.add(v);
      if (this._loggingN && this._loggingP) {
        nData += plog._counter + "," + p.logString(_logMin) + "," + n.logString(_logMin) + "," + v.x + "," + v.y + "," + v.z + "," + v.mag() + "," + distance + "\n";
      }
    }
    if (this._loggingN && this._loggingP) {
      nClog.dump(nData);
      nClog.clean();
    }
    if (p._nbr.size() > 0) {
      vcb.div(p._nbr.size());
    }
    return vcb;
  }

  PVectorD repulsion(Particle p) {
/** 
* repulsion calculation - Calculates the repulsion between each agent and its neigbours.
* 
* @param p The particle that is currently being checked
*/
    PVectorD vrb = new PVectorD();
    PVectorD v = new PVectorD();
    int count = 0;
    double dist = 0.0;
    double distance = 0.0;
    String nData = "";
    for(Particle n : p._nbr) {
      // IF compress permeter then reduce repulsion field if both agents are perimeter agents.
      if (p._isPerim && n._isPerim) { 
        dist = p._Rb * this._pr;
      } else {
        dist = p._Rb;
      }
      distance = PVectorD.dist(p._loc,n._loc);
      if (distance <= dist & p != n) {
        count++;
        v = PVectorD.sub(p._loc, n._loc).setMag(dist - distance).mult(this._kr);
        vrb.add(v);
        if (this._loggingN && this._loggingP) {
          nData += plog._counter + "," + p.logString(this._logMin) + "," + n.logString(this._logMin) + "," + v.x + "," + v.y + "," + v.z + "," + v.mag() + "\n";
        }
      }
    }
    if (this._loggingN && this._loggingP) {
      nRlog.dump(nData);
      nRlog.clean();
    }
    if (count > 0) {
      vrb.div(count);
    }
    return vrb;
  }

  PVectorD direction(Particle p) {
/** 
* direction calculation - Calculates the normalised direction.
* 
* @param p The particle that is currently being checked
*/
    PVectorD destination = new PVectorD(0,0,0);
    PVectorD vd = new PVectorD(0,0,0);
    if (p._destinations.size() > 0) {
      destination = p._destinations.get(0)._loc;      
      for (int i = 1; i < p._destinations.size(); i++) {
        if (PVectorD.dist(p._loc,destination) > PVectorD.dist(p._loc,p._destinations.get(i)._loc)) {
          destination = p._destinations.get(i)._loc;
        }
      }   
    }    
    if (!this._perimCoord) {
      vd = PVectorD.sub(destination,p._loc);
    } else {
      /* Perimeter only control */
      if (p._isPerim) {
        vd = PVectorD.sub(destination,p._loc);
      }
    }
    return vd.setMag(this._kd);
  }

  public PVectorD getCentroid() {
    PVectorD center = new PVectorD(0,0);
    for(Particle p : this.S) {
      center.add(p._loc);
    }    
    center.div(this.S.size());
    return center;
  }

  public void saveSwarm(String filename) {
    try {
      saveSwarmJSON(new PrintWriter(new File("data/json/" + filename)));
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public void saveSwarm() {
    try {
      saveSwarmJSON(new PrintWriter(new File("data/json/" + modelProperties.getProperty("swarmName"))));
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  public void saveSwarmJSON(PrintWriter jsonFile) {
/** 
* Save environment settings to JSON file.
* 
*/  
    JSONObject json = new JSONObject();
    JSONObject jsonParams = new JSONObject();
    JSONObject jsonInfo = new JSONObject();

    try {
      jsonInfo.put("by","JSwarm CLI");
      jsonInfo.put("date",LocalDate.now());
      jsonInfo.put("time",LocalTime.now());
    } catch (JSONException e1) {
      e1.printStackTrace();
    }

    JSONObject jsonAgents = new JSONObject();
//    JSONArray jsonAgentsProps = new JSONArray();
    JSONArray jsonAgentsCoords = new JSONArray();
    JSONArray jsonAgentsX = new JSONArray();
    JSONArray jsonAgentsY = new JSONArray();
    JSONArray jsonAgentsZ = new JSONArray();

    JSONObject jsonDestinations = new JSONObject();
//    JSONArray jsonDestinationsProps = new JSONArray();
    JSONArray jsonDestinationsCoords = new JSONArray();
    JSONArray jsonDestinationsX = new JSONArray();
    JSONArray jsonDestinationsY = new JSONArray();
    JSONArray jsonDestinationsZ = new JSONArray();

    JSONObject jsonObstacles = new JSONObject();
//    JSONArray jsonObstaclesProps = new JSONArray();
    JSONArray jsonObstaclesCoords = new JSONArray();
    JSONArray jsonObstaclesX = new JSONArray();
    JSONArray jsonObstaclesY = new JSONArray();
    JSONArray jsonObstaclesZ = new JSONArray();
    
    try {
      jsonParams.put("cb",this._Cb);
//    jsonParams.put("seed",this._seed);
//    jsonParams.put("grid",this._grid);
      jsonParams.put("rb",this._Rb);
      jsonParams.put("kr",this._kr);
      jsonParams.put("kc",this._kc);
      jsonParams.put("kd",this._kd);
      jsonParams.put("ko",this._ko);
      jsonParams.put("kg",this._kg);
      jsonParams.put("ob",this._Ob);
      jsonParams.put("pr",this._pr);
      jsonParams.put("pc",this._pc);
      jsonParams.put("speed",this._speed);
      jsonParams.put("perim_coord",this._perimCoord);
//  CROSS COMPATABILITY SETTINGS FOR PYTHON MODEL
      jsonParams.put("scaling","linear");
      jsonParams.put("stability_factor", 0.0);
      jsonParams.put("exp_rate", 0.2);


      for(Particle p : S) {
//      jsonAgentsProps.setJSONObject(i,p.getJSONProps());
        jsonAgentsX.put(p._loc.x);
        jsonAgentsY.put(p._loc.y);
        jsonAgentsZ.put(p._loc.z);
      }
    
//    jsonAgentsCoords.put(JSONArray({jsonAgentsX,jsonAgentsY,jsonAgentsZ});
      jsonAgentsCoords.put(jsonAgentsX);
      jsonAgentsCoords.put(jsonAgentsY);
      jsonAgentsCoords.put(jsonAgentsZ);

      jsonAgents.put("coords",jsonAgentsCoords);
//    jsonAgents.put("props",jsonAgentsProps);

      for(Obstacle o : O) {
//      jsonObstaclesProps.setJSONObject(i,o.getJSONProps());
        jsonObstaclesX.put(o._loc.x);
        jsonObstaclesY.put(o._loc.y);
        jsonObstaclesZ.put(o._loc.z);
      } 

      jsonObstaclesCoords.put(jsonObstaclesX);
      jsonObstaclesCoords.put(jsonObstaclesY);
      jsonObstaclesCoords.put(jsonObstaclesZ);

      jsonObstacles.put("coords",jsonObstaclesCoords);
//    jsonObstacles.put("props",jsonObstaclesProps);

      for(Destination d : D) {
//      jsonDestinationsProps.setJSONObject(i,d.getJSONProps());
        jsonDestinationsX.put(d._loc.x);
        jsonDestinationsY.put(d._loc.y);
        jsonDestinationsZ.put(d._loc.z);

      }        

      jsonDestinationsCoords.put(jsonDestinationsX);
      jsonDestinationsCoords.put(jsonDestinationsY);
      jsonDestinationsCoords.put(jsonDestinationsZ);

      jsonDestinations.put("coords",jsonDestinationsCoords);
//    jsonDestinations.put("props",jsonDestinationsProps);
      json.put("obstacles",jsonObstacles);
      json.put("destinations",jsonDestinations);
      json.put("agents",jsonAgents);
      json.put("params",jsonParams);
      json.put("info",jsonInfo);

      jsonFile.println(json.toString(4));
      jsonFile.flush();
    } catch (JSONException e) {
      e.printStackTrace();
    }
  }

  public void loadSwarm(String file) {
    Path fileName = Path.of("data/json/" + file);
    JSONObject json = null;
    try {
      json = new JSONObject(Files.readString(fileName));
    } catch (JSONException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    }
    load(json);  
  }

  public void loadSwarm() {
    Path fileName = Path.of("data/json/" + modelProperties.getProperty("swarmName"));
    JSONObject json = null;
    try {
      json = new JSONObject(Files.readString(fileName));
    } catch (JSONException e) {
      e.printStackTrace();
    } catch (IOException e) {
      e.printStackTrace();
    }
    load(json);  
  }

  public void load(JSONObject json) {
/** 
* Load environment settings from JSON file.
*
*/
    try {    
      JSONObject params = json.getJSONObject("params");
      this._Cb = params.getDouble("cb");
      this._Rb = params.getDouble("rb");
      this._kr = params.getDouble("kr");
      this._kc = params.getDouble("kc");
      this._kd = params.getDouble("kd");
      this._ko = params.getDouble("ko");
      this._kg = params.getDouble("kg");
      this._Ob = params.getDouble("ob");
      this._pr = params.getDouble("pr");
      this._pc = params.getDouble("pc");
      this._speed = params.getDouble("speed");
      this._perimCoord = params.getBoolean("perim_coord");
    } catch (JSONException e1) {
      e1.printStackTrace();
    }

    this.S.clear();

// Commented JSON components to created reduced data set. These might be resurrected later.
//    JSONArray props = json.getJSONObject("agents").getJSONArray("props");
    JSONArray coords = null;
    try {
      coords = json.getJSONObject("agents").getJSONArray("coords");
    } catch (JSONException e2) {
      e2.printStackTrace();
    }
    try {
      for (int i = 0; i < coords.getJSONArray(0).length(); i++) {
//      JSONObject p = props.getJSONObject(i);
        JSONArray x = coords.getJSONArray(0);
        JSONArray y = coords.getJSONArray(1);
        JSONArray z = coords.getJSONArray(2);
        S.add(new Particle(i, (double)x.getDouble(i), (double)y.getDouble(i), (double)z.getDouble(i), this._Cb, this._Rb, this._speed));
        Particle._nextParticleId = i + 1;
      }
    } catch (Exception e2) {
      e2.printStackTrace();
      System.exit(-1);
    }

//    props = json.getJSONObject("destinations").getJSONArray("props");
    try {
      coords = json.getJSONObject("destinations").getJSONArray("coords");
    } catch (JSONException e) {
      e.printStackTrace();
    }

    try {
      for (int i = 0; i < coords.getJSONArray(0).length(); i++) {
//      JSONObject d = props.getJSONObject(i);
        JSONArray x = coords.getJSONArray(0);
        JSONArray y = coords.getJSONArray(1);
        JSONArray z = coords.getJSONArray(2);

        Destination dest = new Destination(i, (double)x.getDouble(i), (double)y.getDouble(i), (double)z.getDouble(i));
        D.add(dest);
        Destination._nextDestId = i + 1;
        for(Particle p : S) {
          p.addDestination(dest);
        }
      }
    } catch (JSONException e) {
      e.printStackTrace();
    }

//    props = json.getJSONObject("obstacles").getJSONArray("props");
    try {
      coords = json.getJSONObject("obstacles").getJSONArray("coords");
    } catch (JSONException e) {
      e.printStackTrace();
    }

    try {
      for (int i = 0; i < coords.getJSONArray(0).length(); i++) {
//      JSONObject o = props.getJSONObject(i);
        JSONArray x = coords.getJSONArray(0);
        JSONArray y = coords.getJSONArray(1);
        JSONArray z = coords.getJSONArray(2);
        this.O.add(new Obstacle(i, (double)x.getDouble(i), (double)y.getDouble(i), (double)z.getDouble(i), this._Ob));
        Obstacle._nextObsId = i + 1;
      }
    } catch (JSONException e) {
      e.printStackTrace();
    } 
// Initialise the swarm based on current model requirements.    
    this.init(); 
  }
  
  public void moveReset() {
    for(Particle p : this.S) {
      p.move();
      p.reset();
    }
  }
  
  public PVectorD avoidObstacles(Particle p) {
/** 
* obstacle avoidance calculation - Calculates the repulsion
* 
* @param p The particle that is currently being checked
*/
    PVectorD result = new PVectorD(0,0,0);
// GET ALL THE IN RANGE OBSTACLES
    for(Obstacle o :this.O) {
      if (PVectorD.dist(p._loc,o._loc) <= o._Ob) {
         result.add(PVectorD.sub(o._loc,p._loc));
      }
    }
    result.add(calcLineRepulsion(p));
    return result.mult(-_ko);
  }

  public PVectorD calcLineRepulsion(Particle p) {
    PVectorD result = new PVectorD(0,0,0);
    if (this.O.size() > 1) {
      for (int i = 1; i <this.O.size(); i++) {
        double x0 = p._loc.x;
        double y0 = p._loc.y;
        double x1 =this.O.get(i)._loc.x;
        double y1 =this.O.get(i)._loc.y;
        double x2 =this.O.get(i-1)._loc.x;
        double y2 =this.O.get(i-1)._loc.y;
        double dir = ((x2-x1) * (y1-y0)) - ((x1-x0) * (y2-y1)); // above or below line segment
        double distance = distBetweenPointAndLine(x0,y0,x1,y1,x2,y2);
        ArrayList<PVectorD> polygon = new ArrayList<PVectorD>();

        PVectorD start =this.O.get(i)._loc;
        PVectorD end =this.O.get(i-1)._loc;
        PVectorD d = PVectorD.sub(end,start);
        d.rotate(Math.PI/2).setMag(this._Ob); 
        polygon.add(PVectorD.add(start,d));
        polygon.add(PVectorD.add(end,d));
        polygon.add(PVectorD.sub(end,d));
        polygon.add(PVectorD.sub(start,d));
        if (distance <= this._Ob && pointInRectangle(p._loc,polygon)) {
          if (dir > 0) {
            result.add(d);
          } else {
            result.sub(d);
          }
        }
      }
    }
    return result;
  }

  public double distBetweenPointAndLine(double x, double y, double x1, double y1, double x2, double y2) {
    // A - the standalone point (x, y)
    // B - start point of the line segment (x1, y1)
    // C - end point of the line segment (x2, y2)
    // D - the crossing point between line from A to BC
    double AB = distBetween(x, y, x1, y1);
    double BC = distBetween(x1, y1, x2, y2);
    double AC = distBetween(x, y, x2, y2);

    // Heron's formula
    double AD;
    double s = (AB + BC + AC) / 2;
    double area = (double) Math.sqrt(s * (s - AB) * (s - BC) * (s - AC));
    AD = (2 * area) / BC;
    return AD;
  }

  public double distBetween(double x, double y, double x1, double y1) {
    double xx = x1 - x;
    double yy = y1 - y;
    return (double) Math.sqrt(xx * xx + yy * yy);
  }

  public boolean pointInRectangle(PVectorD p, ArrayList<PVectorD> polygon) {
    boolean isInside = false;
    for (int i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
        if ( (polygon.get(i).y > p.y) != (polygon.get(j).y > p.y) &&
                p.x < (polygon.get(j).x - polygon.get(i).x) * (p.y - polygon.get(i).y) / (polygon.get(j).y - polygon.get(i).y) + polygon.get(i).x ) {
            isInside = !isInside;
        }
    }
    return isInside;
  }

  public void addDestination(double x, double y, double z) {
/** 
* Add Destination in 3D
* 
* @param x X Position
* @param y Y Position
* @param z Z Position
*/
    Destination d = new Destination(Destination._nextDestId++,(double)x,(double)y,(double)z);
     this.D.add(d);
    for(Particle p : S) {
      p.addDestination(d);
    }
  }

  public void deleteDestination(Destination d) {
/** 
* Delete Destination
* 
* @param d Destination
*/
    for (int i =  this.D.size() - 1; i >= 0; i--) {
      Destination dest =  this.D.get(i);
      if (d == dest) {
        for(Particle p : S) {
          p.removeDestination(d);
        }
         this.D.remove(i);
      }
    }
  }

  public void addParticle(double x, double y, double z) {
/** 
* Add Particle/Agent in 3D
* 
* @param x X Position
* @param y Y Position
* @param z Z Position
*/
    try {
      // create agent in centred quartile.
      Particle p = new Particle(Particle._nextParticleId++, x, y, z, this._Cb, this._Rb, this._speed);
      p.setDestinations( this.D);
      this.S.add(p);
    } catch (Exception e) {
      System.out.println(e);
      System.exit(-1);
    }
  }
  
  public void deleteParticle(Particle p) {
/** 
* Delete Particle/Agent
* 
* @param p Particle
*/
    for (int i = this.S.size() - 1; i >= 0; i--) {
      Particle part = this.S.get(i);
      if (part == p) {
        this.S.remove(i);
      }
    }
  }

  public void deleteObstacle(Obstacle o) {
/** 
* Delete Obstacle
* 
* @param o Obstacle
*/
    for (int i =this.O.size() - 1; i >= 0; i--) {
      Obstacle obs =this.O.get(i);
      if (obs == o) {
       this.O.remove(i);
      }
    }
  }

  public void addObstacle(double x, double y, double z) {
/** 
* Add Destination in 3D
* 
* @param x X Position
* @param y Y Position
* @param z Z Position
*/
   this.O.add(new Obstacle(Obstacle._nextObsId++, x, y, z, this._Ob));
  }

  public boolean hasObstacles() {
    return (this.O.size() > 0);
  }
}
