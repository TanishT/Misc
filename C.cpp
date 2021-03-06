#include <bits/stdc++.h>
using namespace std;
 
typedef long long ll;
typedef pair<int, int> pi;
typedef pair<ll,ll> pl;
typedef vector<int> vi;
typedef vector<ll> vl;
 
#define FOR(i, a, b) for (int i=a; i<(b); i++)
#define F0R(i, a) for (int i=0; i<(a); i++)
#define FORd(i,a,b) for (int i = (b)-1; i >= a; i--)
#define F0Rd(i,a) for (int i = (a)-1; i >= 0; i--)
#define trav(a,x) for (auto& a : x)

#define sz(x) (int)(x).size()
#define pb push_back
#define f first
#define s second
#define lb lower_bound
#define ub upper_bound
#define all(x) x.begin(), x.end()
#define ins insert

const int MOD = 1000000007;

ll modExp(ll base, ll power) {
    if (power == 0) {
        return 1;
    } else {
        ll cur = modExp(base, power / 2); cur = cur * cur; cur = cur % MOD;
        if (power % 2 == 1) cur = cur * base;
        cur = cur % MOD;
        return cur;
    }
}

ll choose[501][501];

void solve() {
    int N, M; cin >> N >> M;
    vector<int> graph[2*N];
    F0R(i, M) {
        int X, Y; cin >> X >> Y; X--; Y--;
        graph[X].pb(Y+N);
        graph[X+N].pb(Y);
        graph[Y].pb(X+N);
        graph[Y+N].pb(X);
    }
    int dist[2*N]; F0R(i, 2*N) dist[i] = 1000000;
    queue<int> q;
    q.push(0); 
    dist[0] = 0;
    while (!q.empty()) {
        int x = q.front(); q.pop();
        trav(a, graph[x]) {
            if (dist[a] > dist[x] + 1) {
                dist[a] = dist[x] + 1; q.push(a);
            }
        }
    }
    if (dist[N] > 2*N+10) {
        ll cnt[2*N+10]; F0R(i, 2*N+10) cnt[i] = 0;
        F0R(i, N) {
            cnt[min(dist[i], dist[i+N])]++;
        }
        ll ans = 1;
        FOR(i, 1, 2*N+10) {
            F0R(j, cnt[i]) {
                ans *= (modExp(2, cnt[i-1]) + MOD - 1) % MOD;
                ans %= MOD;
            }
        }
        cout << ans << endl; return;
    }
    ll ans = 1;
    map<int, int> cnt[4*N+10];
    F0R(i, N) {
        int x = min(dist[i], dist[i+N]); int y = max(dist[i], dist[i+N]);
        cnt[x+y][x]++;
        //cout << x << " " << y << endl;
    }

    F0Rd(m, 4*N+10) {
        vi tbr;
        trav(a, cnt[m]) if (a.s == 0) tbr.pb(a.f);
        trav(a, tbr) cnt[m].erase(a);
        int lst = -100;
        vl dp = {1};
        bool doproc = false;
        trav(a, cnt[m]) {
            if (a.s == 0) continue;
            //cout << m << " " << a.f << " " << a.s << endl;
            if (lst < a.f-1) {
                ll cur = 0;
                trav(a, dp) cur += a;
                cur %= MOD;
                ans *= cur; 
                ans %= MOD;
                dp.clear();
                dp = vl(a.s+1);
                dp[0] = 1;
            }
            lst = a.f;
            if (a.f*2+1 == m) {
                doproc = true; break;
            } else if (a.f == cnt[m].rbegin()->f) {
                break;
            }
            ll cn = cnt[m][a.f+1];
            vl nxt(cn+1);
            ll dow = 0; if (m > 2) dow = cnt[m-2][a.f-1];
            dow = modExp(2, dow) - 1;
            if (a.f == 0) {
                F0R(i, cn+1) {
                    nxt[i] = choose[cn][i];
                }
            } else{
                vl mid(sz(dp));
                F0R(i, sz(dp)) {
                    F0R(j, i+1) {
                        ll val = dp[i] * choose[i][j]; val %= MOD;
                        val *= modExp(dow, sz(dp)-1-j); val %= MOD;
                        mid[j] += val;
                        mid[j] %= MOD;
                    }
                }
                F0R(i, sz(mid)) {
                    ll defWays[cn+1];
                    F0R(j, cn+1) {
                        ll ival = modExp(2, j) - 1;
                        ll val = modExp(ival, i) * modExp(ival+1, sz(mid)-1-i);
                        val %= MOD;
                        ll mul = MOD-1;
                        defWays[j] = val;
                        ll numm = choose[cn][j]; val *= numm; val %= MOD;
                        nxt[j] += val * mid[i]; nxt[j] %= MOD;
                        F0Rd(k, j) {
                            ll cx = choose[j][k] * defWays[k]; cx %= MOD; cx *= mul; cx %= MOD;
                            cx *= numm; cx %= MOD;
                            nxt[j] += cx * mid[i]; nxt[j] %= MOD;
                            mul *= MOD-1; mul %= MOD;
                        }
                    }
                }
            }
            dp = nxt;
            /*if (m == 11) {
                cout << a.f << " TEST" << endl;
                trav(b, dp) {
                    cout << b << endl;
                }
            }*/

        }
        ll cur = 0; 
        if (lst == 0) {
            cur = 1;
        } else if (!doproc) {
            ll dow = 0; if (m > 2) dow = cnt[m-2][lst-1];
            dow = modExp(2, dow) - 1;
            F0R(i, sz(dp)) {
                cur += (modExp(dow, sz(dp)-1) * dp[i]);
                cur %= MOD;
            }
        } else {
            ll ways[sz(dp)];
            F0R(i, sz(dp)) {
                int K = sz(dp)-1;
                ways[i] = modExp(2, (K*K+K)/2);
                ll mul = MOD-1;
                FOR(j, 1, i+1) {
                    ll val = mul * choose[i][j]; val %= MOD;
                    val *= modExp(2, ((K-j)*(K-j+1))/2);
                    ways[i] += val; ways[i] %= MOD;
                    mul *= MOD-1; mul %= MOD;
                }
                
            }
            ll dow = 0; if (m > 2) dow = cnt[m-2][lst-1];
            dow = modExp(2, dow) - 1;
            F0R(i, sz(dp)) {
                F0R(j, i+1) {
                    
                    ll val = choose[i][j] * ways[j]; val %= MOD; val *= dp[i]; val %= MOD;
                    val *= modExp(dow, sz(dp)-1-j); val %= MOD;
                    cur += val; cur %= MOD;
                }
            }
        }
        ans *= cur;
        ans %= MOD;
    }

    cout << ans << endl;
}

int main() {
	ios_base::sync_with_stdio(0); cin.tie(0);    
    F0R(i, 501) {
        F0R(j, 501) {
            if (i < j) {
                choose[i][j] = 0;
            } else if (j == 0) {
                choose[i][j] = 1;
            } else {
                choose[i][j] = choose[i-1][j-1] + choose[i-1][j];
                choose[i][j] %= MOD;
            }
        }
    }


    int T; cin >> T;
    while(T--) {
        solve();
    }
	
	return 0;
}
 


