from locale import normalize

import numpy as np
from scipy.odr import quadratic

from AdaptiveController_ElbowJointFES.Program.Sub.Normalization_functions import normalize, scale_output

# Creating nonlinear monotonically increasing functions

def linear(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = x_norm
    y_norm = normalize(y, 0, 1)
    return scale_output(y_norm, min_val, max_val)

def log(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = np.log1p(x_norm)
    y_norm = normalize(y, np.log1p(0), np.log1p(1))
    return scale_output(y_norm, min_val, max_val)

def x2(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = x_norm**2
    y_norm = normalize(y, 0 ** 2, 1 ** 2)
    return scale_output(y_norm, min_val, max_val)

def x3(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = x_norm**3
    y_norm = normalize(y, 0 ** 3, 1 ** 3)
    return scale_output(y_norm, min_val, max_val)

def FOD5(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = 1 - np.exp(-5 * x_norm)
    y_norm = normalize(y, 1 - np.exp(-5 * 0), 1 - np.exp(-5 * 1))
    return scale_output(y_norm, min_val, max_val)

def tanh(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = np.tanh(3 * x_norm)
    y_norm = normalize(y, np.tanh(3 * 0), np.tanh(3 * 1))
    return scale_output(y_norm, min_val, max_val)

def sqrt(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = np.sqrt(x_norm)
    y_norm = normalize(y, np.sqrt(0), np.sqrt(1))
    return scale_output(y_norm, min_val, max_val)

def sin2(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = np.sin(np.pi * x_norm / 2)**2
    y_norm = normalize(y, np.sin(np.pi * 0 / 2) ** 2, np.sin(np.pi * 1 / 2) ** 2)
    return scale_output(y_norm, min_val, max_val)

def exponential(x, min_val, max_val, k=0.3):
    x_norm = normalize(x, min_val, max_val)
    y = np.exp(k * x_norm) - 1
    y_norm = normalize(y, np.exp(k * 0) - 1, np.exp(k * 1) - 1)
    return scale_output(y_norm, min_val, max_val)

def FOD4(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = 1 - np.exp(-4 * x_norm)
    y_norm = normalize(y, 1 - np.exp(-4 * 0), 1 - np.exp(-4 * 1))
    return scale_output(y_norm, min_val, max_val)

def FOD3(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = 1 - np.exp(-3 * x_norm)
    y_norm = normalize(y, 1 - np.exp(-3 * 0), 1 - np.exp(-3 * 1))
    return scale_output(y_norm, min_val, max_val)

def FOD2p5(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = 1 - np.exp(-2.5 * x_norm)
    y_norm = normalize(y, 1 - np.exp(-2.5 * 0), 1 - np.exp(-2.5 * 1))
    return scale_output(y_norm, min_val, max_val)

def FOD2(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = 1 - np.exp(-2 * x_norm)
    y_norm = normalize(y, 1 - np.exp(-2 * 0), 1 - np.exp(-2 * 1))
    return scale_output(y_norm, min_val, max_val)

def FOD1p5(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = 1 - np.exp(-1.5 * x_norm)
    y_norm = normalize(y, 1 - np.exp(-1.5 * 0), 1 - np.exp(-1.5 * 1))
    return scale_output(y_norm, min_val, max_val)

def FOD1(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = 1 - np.exp(-1 * x_norm)
    y_norm = normalize(y, 1 - np.exp(-1 * 0), 1 - np.exp(-1 * 1))
    return scale_output(y_norm, min_val, max_val)

def FOD0p5(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = 1 - np.exp(-0.5 * x_norm)
    y_norm = normalize(y, 1 - np.exp(-0.5 * 0), 1 - np.exp(-0.5 * 1))
    return scale_output(y_norm, min_val, max_val)

def x1p5(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = np.sqrt(x_norm) * x_norm
    y_norm = normalize(y, np.sqrt(0) * 0, np.sqrt(1) * 1)
    return scale_output(y_norm, min_val, max_val)

def x1p07(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = x_norm ** 1.07
    y_norm = normalize(y, 0, 1)
    return scale_output(y_norm, min_val, max_val)

def x1p3(x, min_val, max_val):
    x_norm = normalize(x, min_val, max_val)
    y = x_norm ** 1.3
    y_norm = normalize(y, 0, 1)
    return scale_output(y_norm, min_val, max_val)

def rec1(x, min_val, max_val):
    coeffi_list = [-843.6979511277019, 5197.536430412725, -13412.638372483592, 18947.706085894395, -16020.186766034052, 8296.16327744554, -2573.059601681965, 445.2229210593829, -39.38236779104095, 3.3340711778271035, 0.0019379696867285955]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec2(x, min_val, max_val):
    coeffi_list = [-5478.148034989951, 29313.84336493856, -67132.46067376861, 85673.25701387382, -66395.62968169135, 31878.612143254002, -9246.193022390822, 1499.4295516492423, -116.37526192868407, 4.663323925164421, -0.001085733743120492]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec3(x, min_val, max_val):
    coeffi_list = [983.6295620230256, -4147.807053645421, 7376.870657315908, -7326.63997886756, 4593.729554123696, -1973.7752629713357, 608.799364929629, -129.5769008142586, 15.205461396257459, 0.5495750522246542, 0.0032584850882765486]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec4(x, min_val, max_val):
    coeffi_list = [2598.8050301385542, -12975.477486500951, 27649.51493869647, -32703.25294368314, 23374.490414060427, -10313.14941842747, 2771.845976389467, -440.81429127115643, 38.64079617739035, 0.4018653227267035, 0.0019438958260757006]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec5(x, min_val, max_val):
    coeffi_list = [1227.5804517280894, -4838.9917490157795, 7204.6762211153955, -4482.477414433996, 95.03285609262137, 1433.0975406028783, -829.7205362934843, 218.3843759110477, -28.63059848142621, 2.0521671283604626, -0.0005214853855697188]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec6(x, min_val, max_val):
    coeffi_list = [9605.660692204787, -48511.220882308844, 103768.50200627546, -122352.4397371199, 86738.75273354688, -37866.39951392221, 10016.81379725474, -1504.132423140846, 105.46325480663576, 0.0034918382914830726, 0.003949390422303285]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec7(x, min_val, max_val):
    coeffi_list = [-942.3620853927717, 4844.552120756491, -10123.078521595988, 10999.402619071894, -6519.336879708741, 1923.9558026556936, -130.94046764584692, -62.22432497921559, 8.236997544441547, 2.7957529672827532, 0.0003396356768374423]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec8(x, min_val, max_val):
    coeffi_list = [-1669.3819771716264, 8448.131465678794, -18407.817554853184, 22789.431225483688, -17793.386812705234, 9106.233754352867, -3004.05249488681, 581.3742165403705, -51.54544655406942, 2.021474891353077, -0.0076302805485602195]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec9(x, min_val, max_val):
    coeffi_list = [893.5815259953605, -4946.283801601582, 11591.67328108179, -15066.165051588501, 11986.395184623469, -6090.4785178337415, 1985.1318628572478, -388.7595977278723, 34.44593810579824, 1.455738472695718, 0.003898414153999275]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec10(x, min_val, max_val):
    coeffi_list = [5844.956865417166, -29810.40923022608, 64104.98363879891, -75502.19816372782, 52993.801361261525, -22634.699956656194, 5773.164257979211, -825.1945265362897, 57.79719121811309, -1.211626290136983, 0.008652784848360406]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec11(x, min_val, max_val):
    coeffi_list = [-3216.846999634267, 16897.826733124308, -37469.31054615323, 45478.4966769205, -32746.008087550912, 14167.651206542534, -3562.678155766245, 479.6848229478897, -29.32883977848012, 1.51903668414551, -0.004196408212841315]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec12(x, min_val, max_val):
    coeffi_list = [-4257.677526774673, 20773.764062872277, -42420.9316903135, 46907.38942049049, -30346.89360004972, 11598.572236439524, -2534.6493960661023, 295.0274661363645, -14.478583375330658, 0.8716957555381832, -3.202320971202243e-05]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec13(x, min_val, max_val):
    coeffi_list = [-6846.184925135634, 33656.39161875443, -69391.203736441, 77624.70444799638, -50866.179164121386, 19652.68180738285, -4288.478380145314, 480.51431251444546, -22.443547430418956, 1.18947039770375, -0.0015332127887487338]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec14(x, min_val, max_val):
    coeffi_list = [5373.1152562636125, -28097.174394010064, 62041.23208690461, -75030.34049987007, 53853.42513207704, -23195.762323664465, 5760.386189986701, -748.6807609290869, 45.25914395781185, -0.46201282807866073, 0.006493705241466796]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec15(x, min_val, max_val):
    coeffi_list = [-1556.3779181423158, 8242.179239372064, -18452.111411878384, 22683.735528937796, -16629.576655750036, 7369.165950856427, -1896.3884142483341, 254.2692908097262, -16.082265651658155, 2.1904157267061417, -0.0036213086155579993]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec16(x, min_val, max_val):
    coeffi_list = [-6156.841463371421, 31466.37058646565, -68078.30926423232, 81215.85543789773, -58341.0382431887, 25876.183731847985, -6983.873579799917, 1085.9103467204513, -86.46585983319304, 3.218542521542437, -0.005953288887101802]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec17(x, min_val, max_val):
    coeffi_list = [-1891.4815881560294, 9655.327430332905, -20377.915384758464, 23033.123947938824, -15122.28919822203, 5894.432452887599, -1371.189280682408, 197.10637255436166, -17.08554580009065, 0.9715032501299722, 0.0015653843216114323]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec18(x, min_val, max_val):
    coeffi_list = [-3757.6719893095096, 16903.978334426058, -31661.912807881312, 32133.628528785033, -19353.378295463972, 7173.767508812647, -1654.9866801860833, 237.033727771583, -20.930071300803384, 1.4658814673336544, 0.0009056964141951174]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec19(x, min_val, max_val):
    coeffi_list = [598.0128113358427, -2217.870049181841, 2986.4588178294107, -1414.3172051225663, -506.6632537306256, 880.9522346344808, -404.3746472495731, 88.51999625106228, -11.870486281599655, 2.160620451125094, -0.0031622709341408844]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

def rec20(x, min_val, max_val):
    coeffi_list = [1266.1676588293085, -6675.176352045187, 15025.984439509763, -18822.675954181275, 14307.580245915333, -6725.637079839257, 1897.5650964248546, -292.58484240753586, 17.380524164269172, 2.399594862662588, -0.002401884279169703]
    x_norm = normalize(x, min_val, max_val)
    def Rec(coeffi_list, x):
        return (coeffi_list[0] * (x**10) + coeffi_list[1] * (x**9) + coeffi_list[2] * (x**8) + coeffi_list[3] * (x**7) + coeffi_list[4] * (x**6) + coeffi_list[5] * (x**5) + coeffi_list[6] * (x**4) + coeffi_list[7] * (x**3) + coeffi_list[8] * (x**2) + coeffi_list[9] * (x**1) + coeffi_list[10])
    y = Rec(coeffi_list, x_norm)
    y_norm = normalize(y, Rec(coeffi_list, 0), Rec(coeffi_list, 1))
    return scale_output(y_norm, min_val, max_val)

