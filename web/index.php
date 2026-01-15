<!doctype html>
<html>
<head>
<meta charset="UTF-8">
<title>Jovian UK - Ground Station</title>
<link rel="stylesheet" href="https://use.typekit.net/jtt4ncl.css">
<style>
	body{
		/*#F6541E*/
		background: #747373;
		width: 100vw;
		height: 100vh;
		margin: 0;
		padding: 0;
		
		display: flex;
		flex-direction: column;
		justify-content: flex-start;
		align-items: center;
		font-family: ui-monospace, Consolas, monospace;
		color: #cfd7e0;
	}	
	
	.navBar{
		background: #27201D;
		height: 50px;
		width: 90%;
		
		border-radius: 0 0 7.5px 7.5px;
		padding: 7.5px;
		
		box-shadow: 6px 6px 16px 1px rgba(0,0,0,0.75);
		-webkit-box-shadow: 6px 6px 16px 1px rgba(0,0,0,0.75);
		-moz-box-shadow: 6px 6px 16px 1px rgba(0,0,0,0.75);
	}
	
	.title{
		color: #F6541E;
		height: 100%;
		display: flex;
		justify-content: flex-start;
		align-items: center;
		margin-left: 15px;
		
		font-family: "baudot", sans-serif;
		font-weight: 400;
		font-style: normal;
		
		font-size: 2em;
	}
	
	.imageContainer{
		margin: 15px 7.5px;
		width: 100%;
		display: flex;
		flex-direction: row;
		justify-content: center;
		align-items: center;
	}
	
	.subImageContainer{
		margin: 7.5px;
		display: flex;
		flex-direction: column;
		justify-content: center;
		align-items: center;
		position: relative;
	
	}
	
	.image{
		height: 500px;
		border-radius: 3.75px;
		
		background: black;
		transform: rotate(180deg);
	}
	
	.imageTitle{
		position: absolute;
		top: 0;
		left: 0;
		
		margin: 7.5px 15px;
		
		mix-blend-mode: difference;
		
		font-family: "baudot", sans-serif;
		font-weight: 400;
		font-style: normal;
		
		font-size: 2em;
		
		color: white;
	}
	
	.terminalContainer{
		background: #27201D;
		border-radius: 7.5px;
		overflow:auto; white-space:pre-wrap; 
		padding: 0 7.5px;
		width: calc(100% - 15px);
		height: calc(100% - 15px);
		margin: 0 0 15px 0;
	}
	
	#bar { 
		display: flex;
		flex-direction: row;
		padding: 0 7.5px;
		height: 50px;
		width: calc(100% - 15px);
		border-radius: 7.5px;
		
		background:#27201D; 
	}
	#input{
		flex: 1;
		height: 50px;
		background: transparent;
		border: none;
		color: white;
		font-size: 1.125em;
	}
	
	.bottomL{
		height: 100%;
		display: flex;
		flex-direction: column;
		justify-content: flex-start;
		align-items: center;
		width: 75%
	}
	.bottomR{
		height: 100%;
		width: calc(25% - 15px);
		background: #27201D;
		
		margin: 0 0 0 15px;
		
		border-radius: 7.5px;
		
		overflow-y: scroll;
		overflow-x: hidden;
	}
	
	.bottomSec{
		display: flex;
		flex-direction: row;
		justify-content: center;
		align-items: center;
		
		
		width: calc(100% - 45px);
		height: calc(300px - 30px);
		margin: 0 15px 15px 15px;
	}
	
</style>
</head>

<body>
	<div class="navBar">
		<div class="title">[BRUCE] SpringSat - Ground Station</div>
	</div>
	<div class="imageContainer">
		<div class="subImageContainer">
			<img src="media/left.jpg" class="image" id="leftImage">
			<div class="imageTitle">Left</div>
		</div>
		<div class="subImageContainer">
			<img src="media/right.jpg" class="image" id="rightImage">
			<div class="imageTitle">Right</div>
		</div>
	</div>
	<div class="bottomSec">
		<div class="bottomL">
		
			<div class="terminalContainer" id="log">
			</div>

		  <div id="bar">
			<input id="input" placeholder="Commands: tlm <id> | cmd <id> | raw <hex> | help" />
		  </div>

			
		</div>
		<div class="bottomR">
		
			
	<div class="stats">
<h3>CPU</h3>
<div>Temp: <span data-sensor="0x0003" data-path="cpu_temp">—</span> °C</div>

<h3>Euler (0x2020)</h3>
<div>Heading: <span data-sensor="0x2020" data-path="euler_deg.heading" data-fmt="2">—</span></div>
<div>Roll:    <span data-sensor="0x2020" data-path="euler_deg.roll"    data-fmt="2">—</span></div>
<div>Pitch:   <span data-sensor="0x2020" data-path="euler_deg.pitch"   data-fmt="2">—</span></div>

<h3>Quaternion (0x2030)</h3>
<div>W: <span data-sensor="0x2030" data-path="quat.w" data-fmt="4">—</span></div>
<div>X: <span data-sensor="0x2030" data-path="quat.x" data-fmt="4">—</span></div>
<div>Y: <span data-sensor="0x2030" data-path="quat.y" data-fmt="4">—</span></div>
<div>Z: <span data-sensor="0x2030" data-path="quat.z" data-fmt="4">—</span></div>

<h3>Calibration (0x2010)</h3>
<div>SYS:   <span data-sensor="0x2010" data-path="calibration.sys">—</span></div>
<div>GYRO:  <span data-sensor="0x2010" data-path="calibration.gyro">—</span></div>
<div>ACCEL: <span data-sensor="0x2010" data-path="calibration.accel">—</span></div>
<div>MAG:   <span data-sensor="0x2010" data-path="calibration.mag">—</span></div>

<h3>Gravity (0x2040)</h3>
<div>X: <span data-sensor="0x2040" data-path="gravity_mps2.x" data-fmt="3">—</span></div>
<div>Y: <span data-sensor="0x2040" data-path="gravity_mps2.y" data-fmt="3">—</span></div>
<div>Z: <span data-sensor="0x2040" data-path="gravity_mps2.z" data-fmt="3">—</span></div>

<h3>Linear Accel (0x2050)</h3>
<div>X: <span data-sensor="0x2050" data-path="linacc_mps2.x" data-fmt="3">—</span></div>
<div>Y: <span data-sensor="0x2050" data-path="linacc_mps2.y" data-fmt="3">—</span></div>
<div>Z: <span data-sensor="0x2050" data-path="linacc_mps2.z" data-fmt="3">—</span></div>
	</div>
			
			
		</div>
	</div>


  <script src="app.js"></script>
</body>
</html>
