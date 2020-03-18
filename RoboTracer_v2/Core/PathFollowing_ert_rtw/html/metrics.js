function CodeMetrics() {
	 this.metricsArray = {};
	 this.metricsArray.var = new Array();
	 this.metricsArray.fcn = new Array();
	 this.metricsArray.var["rtDW"] = {file: "C:\\Users\\robot\\Documents\\MATLAB\\2020_RoboTrace\\PathFollowing\\coder\\PathFollowing_ert_rtw\\PathFollowing.c",
	size: 48};
	 this.metricsArray.var["rtM_"] = {file: "C:\\Users\\robot\\Documents\\MATLAB\\2020_RoboTrace\\PathFollowing\\coder\\PathFollowing_ert_rtw\\PathFollowing.c",
	size: 2};
	 this.metricsArray.var["rtU"] = {file: "C:\\Users\\robot\\Documents\\MATLAB\\2020_RoboTrace\\PathFollowing\\coder\\PathFollowing_ert_rtw\\PathFollowing.c",
	size: 48};
	 this.metricsArray.var["rtY"] = {file: "C:\\Users\\robot\\Documents\\MATLAB\\2020_RoboTrace\\PathFollowing\\coder\\PathFollowing_ert_rtw\\PathFollowing.c",
	size: 16};
	 this.metricsArray.fcn["PathFollowing.c:CalcError1"] = {file: "C:\\Users\\robot\\Documents\\MATLAB\\2020_RoboTrace\\PathFollowing\\coder\\PathFollowing_ert_rtw\\PathFollowing.c",
	stack: 32,
	stackTotal: 32};
	 this.metricsArray.fcn["PathFollowing_initialize"] = {file: "C:\\Users\\robot\\Documents\\MATLAB\\2020_RoboTrace\\PathFollowing\\coder\\PathFollowing_ert_rtw\\PathFollowing.c",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["PathFollowing_step"] = {file: "C:\\Users\\robot\\Documents\\MATLAB\\2020_RoboTrace\\PathFollowing\\coder\\PathFollowing_ert_rtw\\PathFollowing.c",
	stack: 40,
	stackTotal: 72};
	 this.metricsArray.fcn["cos"] = {file: "C:\\Program Files\\MATLAB\\R2019a\\sys\\lcc\\include\\math.h",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["sin"] = {file: "C:\\Program Files\\MATLAB\\R2019a\\sys\\lcc\\include\\math.h",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["sqrt"] = {file: "C:\\Program Files\\MATLAB\\R2019a\\sys\\lcc\\include\\math.h",
	stack: 0,
	stackTotal: 0};
	 this.getMetrics = function(token) { 
		 var data;
		 data = this.metricsArray.var[token];
		 if (!data) {
			 data = this.metricsArray.fcn[token];
			 if (data) data.type = "fcn";
		 } else { 
			 data.type = "var";
		 }
	 return data; }; 
	 this.codeMetricsSummary = '<a href="PathFollowing_metrics.html">Global Memory: 114(bytes) Maximum Stack: 40(bytes)</a>';
	}
CodeMetrics.instance = new CodeMetrics();
