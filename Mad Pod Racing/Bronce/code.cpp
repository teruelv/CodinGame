// DONE: rotating the angle
// DONE: save record of checkpoints
// DONE: added ccw
// DONE: moving to 2D struct
//  50%: tuning thrust function
//  10%: turn before arriving (drift)
// TODO: moving to 2D thrust
// TODO: check if projection is inside pod
// TODO: try different new targets
// TODO: simulate several steps ahead

#include <bits/stdc++.h>
#include <iomanip>

using namespace std;

typedef tuple<int, int, string, string> tri;
typedef tuple<int, int, int> i3;
typedef pair<int, int> i2;
typedef float ld;

const ld PI = acos(static_cast<ld>(-1));
const ld PI2 = PI * 2.0;
const ld MAX_TURN = PI * 0.1;
const ld eps = 1e-6;
const ld Friction = 0.85f;
const ld invFrict = 1.0f / 0.85f;

mt19937 rng(chrono::high_resolution_clock::now().time_since_epoch().count());

static int rnd(int lo, int hi) {
  return uniform_int_distribution<int>(lo, hi)(rng);
}

int sq(int x) { return x * x; }
int sign(double x) { return abs(x) < eps ? 0 : x > 0 ? 1 : -1; }

int ccw(ld x1, ld y1, ld x2, ld y2, ld x3, ld y3) {
  return sign((y2 - y1) * (x3 - x1) - (x2 - x1) * (y3 - y1));
}

ld AtoR(ld A) {
  if (A < 0)
    A += 360;
  return PI * A / 180.0;
}

ld RtoA(ld A) { return A * 180.0 / PI; }

int timer = 0;

struct point {
  ld x, y;
  point(ld _x = 0, ld _y = 0) : x(_x), y(_y) {}
  ld mod() { return sqrt(x * x + y * y); }
  point operator+(point a) { return {x + a.x, y + a.y}; }
  point operator-(point a) { return {x - a.x, y - a.y}; }
  point operator*(ld a) { return {x * a, y * a}; }
  bool operator==(point &a) const { return x == a.x && y == a.y; }
  bool operator!=(point &a) const { return x != a.x || y != a.y; }
  ld dist(point p2) { return ((*this) - p2).mod(); }
};

int ccw(point a, point b, point c) {
  return sign((b.y - a.y) * (c.x - a.x) - (b.x - a.x) * (c.y - a.y));
}

point closestPointToLine(point a, point b, point p) {
  point a_to_p = p - a;
  point a_to_b = b - a;
  ld atb2 = (a - b).mod();
  atb2 *= atb2;

  if (atb2 == 0)
    return p;

  ld atp_dot_atb = a_to_p.x * a_to_b.x + a_to_p.y * a_to_b.y;
  ld t = atp_dot_atb / atb2;

  return point(a.x + a_to_b.x * t, a.y + a_to_b.y * t);
}

struct ship {
  point p, v;
  ld &x, &y;
  ld &vx, &vy;
  // ld vx, vy;
  ld ang, m;
  ship() : x(p.x), y(p.y), vx(v.x), vy(v.y) {};
  ship(point _p) : p(_p), x(p.x), y(p.y), vx(v.x), vy(v.y) {
    ang = m = 0;
  }
  ship(int _x, int _y) : p(_x, _y), x(p.x), y(p.y), vx(v.x), vy(v.y) {
    ang = m = 0;
  }
  ship &operator=(const ship &a) {
    if (this != &a) {
      p = a.p;
      v = a.v;
      m = a.m;
      ang = a.ang;
    }
    return *this;
  }
  bool insideCircle(point p2, ld r = 600) { return (p - p2).mod() <= r; }
  ld getDist(ld tx, ld ty) {
    ld dx = tx - x;
    ld dy = ty - y;
    return sqrt(dx * dx + dy * dy);
  }
  ld getAngle(ld tx, ld ty) {
    ld dx = tx - x;
    ld dy = ty - y;
    ld a = atan2(dy, dx);
    if (a < 0)
      a = PI2 + a;
    return a;
  }
  ld getAngD(ld a) {
    ld LT = ang >= a ? ang - a : PI2 + ang - a;
    ld RT = ang <= a ? a - ang : PI2 - ang + a;
    return LT < RT ? -LT : RT;
  }
  ld tune1(ld da) {
    if (da < -MAX_TURN)
      da = -MAX_TURN;
    if (da > +MAX_TURN)
      da = +MAX_TURN;
    return da;
  }
  ld tuneAngle(ld na) {
    if (na < 0)
      na += PI2;
    return na;
  }
  ship nxt(point _p, ld ac) {
    auto [tx, ty] = _p;
    ship r = *this;
    cerr << "Before:\n";
    r.print();
    r.ang = r.tuneAngle(r.ang + tune1(r.getAngD(r.getAngle(tx, ty))));
    r.vx += cos(r.ang) * ac;
    r.vy += sin(r.ang) * ac;
    r.p = r.p + point(r.vx, r.vy);
    r.x = round(r.x);
    r.y = round(r.y);
    r.vx = int(r.vx * Friction);
    r.vy = int(r.vy * Friction);
    r.m = (p - r.p).mod();
    cerr << "After:\n";
    r.print();
    return r;
  }
  void turn(ld tx, ld ty, ld an) {
    an = AtoR(an);
    tx /= cos(an);
    ty /= sin(an);
  }
  void move(point _p, int ac) { (*this) = nxt(_p, ac); }
  void print() {
    cerr << "Is at: " << (int)x << ", " << (int)y << '\n';
    cerr << "Current angle: " << int(RtoA(ang)) << '\n';
    cerr << "Components are: " << (int)vx << ' ' << (int)vy << '\n';
  }
};

vector<point> pod;

int can_turn = 2;
int checkpoint = 0;
int laps = 1, sz = 0, boost = 1;

ld Fix(ld A) {
  // ld v = A * A / 3600;
  return AtoR(A);
}

ship fix() {
  ship r;
  return r;
}

tri Play(ship &S, point p1, point p2, int D, int A) {
  if (S.x == -1e9) {
    S = ship(p1);
    S.ang = S.getAngle(p2.x, p2.y);
    pod.push_back(p2);
  }

  // if (S.p != p1)
  //   S.p = p1;

  if (p2 != pod[checkpoint]) {
    checkpoint++;
    if (p2 == pod[0])
      laps++, checkpoint = 0;
    if (laps == 1)
      pod.push_back(p2);
  }

  auto r = p2;
  sz = pod.size();

  // cerr << "Lap: " << laps << "/3\n";
  // cerr << "Checkpoint: " << checkpoint + 1 << "\n";
  // cerr << "Total chck: " << sz << "\n";

  int th = 0;
  A = abs(A);

  if (A < 90)
    th = ceil(100.f * cos(A / 180));

  if (laps > 1) {
    int g = -ccw(p1, p2, pod[(checkpoint + 1) % sz]);
    if (g == +0)
      cerr << "Moving colinear\n";
    if (g == -1)
      cerr << "Should turn left\n";
    if (g == +1)
      cerr << "Should turn right\n";
    // auto pp = pod[(checkpoint + 1) % sz];
    // ld D1 = (S.nxt(pp, th).p - p2).mod();
    // ld D2 = (S.nxt(p2, th).p - p2).mod();
    // if (D1 < 1500 && D1 < D2 && sign(g) == sign(A)) {
    //   cerr << "Turning!\n";
    //   r = pp;
    // }
  }

  // if (S.m > 50 && S.ang < 70.f &&
  //     S.nxt().p.dist(currentCheckpoint) <
  //         currentPos.DistanceTo(currentCheckpoint)) {
  //   Vec2 aux = closestPointToLine(currentPos, currentCheckpoint, futurePos);
  //   Vec2 aux2 = aux + (aux - futurePos);
  //   outputPos = aux2;
  // }

  if (laps == 3 && (checkpoint + 1) == pod.size() && boost && A == 0) {
    th = 650;
    boost--;
  }

  // point t = S.tuneThrust(p2, 100);
  point t = point(th, th);

  S.move(p2, t.x);
  cerr << "Moved: " << S.m << '\n';

  if (th == 650)
    return {r.x, r.y, "BOOST", "BOOST"};

  return {r.x, r.y, to_string(int(t.x)), to_string(int(t.y))};
}

int main() {
  // cerr << acos(0.5) << '\n';
  // return 0;
  ship S(-1e9, -1e9);
  // game loop
  while (1) {
    // direction of the next checkpoint
    int px, py; // x,y ship's position
    int nx, ny; // x,y position of the next check point
    int D, A;   // distance,angle to the next checkpoint
    cin >> px >> py >> nx >> ny >> D >> A;
    cin.ignore();
    int opponent_x;
    int opponent_y;
    cin >> opponent_x >> opponent_y;
    cin.ignore();

    // cerr << "Pos: " << px << ' ' << py << '\n';
    cerr << setprecision(2) << fixed;
    cerr << "Data: " << D << ' ' << A << '\n';

    auto [x, y, z1, z2] = Play(S, point(px, py), point(nx, ny), D, A);

    cout << x << " " << y << " " << z1 << ' ' << z2 << endl;
  }
}
