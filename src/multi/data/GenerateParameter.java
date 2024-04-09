package multi.data;

import multi.DefaultSetting;
import multi.structure.*;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;
public class GenerateParameter extends DefaultSetting {
	private Parameter p;
	private InputData in;
	private int timeHorizon;
	private double uncertainDegree = defaultUncertainDegree;

	public GenerateParameter(Parameter p, InputData in, int timeHorizon, double uncertainDegree) throws IOException {
		super();
		this.p = p;
		this.in = in;
		this.timeHorizon=timeHorizon;
		this.uncertainDegree=uncertainDegree;
		random.setSeed(randomSeed);
		frame();
	}

	private void frame() throws IOException {
		System.out.println("========"+ "Start to Generate parameters" + "========");
		double start = System.currentTimeMillis();

		p.setTimeHorizon(timeHorizon);
		p.setTau((int) (Math.sqrt(in.getRequestSet().size()) * budgetCoefficient));
		p.setUncertainDegree(uncertainDegree);
		in.setUncertainDegree(uncertainDegree);

		SetArcSet();

		SetTimePoint();

		// set cost for ports
		SetPorts();

		// set Request (normal/variation demand and penalty cost  for each od pairs)
		SetRequests();

		SetShipRoutes();

		SetVessels();

		SetVesselPaths();

		SetContainerPaths();

		SetArcCapacity();

		SetInitialEmptyContainers();

		if(WhetherGenerateSamples){
			GenerateRandomSampleSceneSet();
		}
		if(WhetherLoadSampleTests){
			p.setSampleScenes(in.getSampleScenes());
		}

		double end = System.currentTimeMillis();
		System.out.println("========"+ "End Generate parameters" + "(" +String.format("%.2f", (end - start)) + "ms)"+ "========");
	}

	public static double getRandDouble()
	{
		double mean = 0.5;
		double variance = 1.0/12.0;
		if(distributionType.equals( "Uniform"))
			return random.nextDouble();
		else if(distributionType.equals( "Normal"))
			return random.nextGaussian();
		else if(distributionType.equals("Log-Normal")){
			// Log-Normal distribution :
			// mean = exp(mu + sigma^2/2)
			// variance = (exp(sigma^2) - 1) * exp(2 * mu + sigma^2)
			double sigma = Math.sqrt(Math.log(1 + variance)) * log_normal_sigma_factor;
			double mu = Math.log(mean) - 0.5 * sigma * sigma;
			// 返回服从Log-normal分布的近似随机数：standard normal --> log-normal
			// z ~ N(0,1) --> X = exp(mu + sigma * z)
			return Math.exp(mu + sigma * random.nextGaussian());
		}
		else
			return random.nextDouble();
	}

	private void SetArcSet(){
		// set travel arc IDs set
		int [] travellingArcsSet =new int[in.getTravelingArcSet().size()];
		int x=0;
		for(TravelingArc tt:in.getTravelingArcSet())
		{
			travellingArcsSet[x]=tt.getTravelingArc_ID();
			x=x+1;
		}
		p.setTravelingArcsSet(travellingArcsSet);

		// set transship arc IDs set
		int [] transhipmentArcsSet =new int [in.getTransshipArcSet().size()];
		x=0;
		for(TransshipArc tt: in.getTransshipArcSet())
		{
			transhipmentArcsSet[x]=tt.getTransshipArc_ID();
			x=x+1;
		}
		p.setTranshipmentArcsSet(transhipmentArcsSet);
	}
	private void SetTimePoint(){
		// set time point set = {0, 1, 2, ..., T}
		int [] timePointSet =new int [timeHorizon+1];
		for(int i=0;i<timeHorizon+1;i++)
		{
			timePointSet[i]=i;
		}
		p.setTimeHorizon(timeHorizon);
		p.setTimePointSet(timePointSet);
	}
	private void SetPorts(){
		// set rentalCost
		// set turnover time (sp)
		// and the demurrage cost of unit laden/empty cost
		// for each port
		// sp = 14
		// unit laden demurrage = 175
		// unit empty demurrage = 100
		// set rental cost of one container per unit time
		p.setRentalCost(DefaultUnitRentalCost);
		String [] portSet =new String [in.getPortSet().size()];
		int [] turnOverTime =new int [in.getPortSet().size()];
		double [] ladenDemurrageCost =new double[in.getPortSet().size()];
		double [] emptyDemurrageCost=new double[in.getPortSet().size()];
		int x=0;
		for(Port pp :in.getPortSet())
		{
			portSet[x]=pp.getPort();
			turnOverTime[x]=DefaultTurnOverTime;
			ladenDemurrageCost[x]=DefaultLadenDemurrageCost;
			emptyDemurrageCost[x]=DefaultEmptyDemurrageCost;
			pp.setRentalCost(DefaultUnitRentalCost);
			pp.setTurnOverTime(DefaultTurnOverTime);
			pp.setLadenDemurrageCost(DefaultLadenDemurrageCost);
			pp.setEmptyDemurrageCost(DefaultEmptyDemurrageCost);
			pp.setLoadingCost(DefaultUnitLoadingCost);
			pp.setDischargeCost(DefaultUnitDischargeCost);
			pp.setTransshipmentCost(DefaultUnitTransshipmentCost);
			x=x+1;
		}
		p.setPortSet(portSet);
		p.setTurnOverTime(turnOverTime);
		p.setLadenDemurrageCost(ladenDemurrageCost);
		p.setEmptyDemurrageCost(emptyDemurrageCost);
	}
	private void WriteRequests() throws IOException {
		String Instance ="S" + randomSeed + "-" +"T"+ p.getTimeHorizon() + "(" + distributionType + ")"+ log_normal_sigma_factor;
		File file = new File(RootPath + DataPath + CasePath + "Demands" + Instance + ".txt");
		if (!file.exists()) {
			try {
				file.createNewFile();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}
		FileWriter filewriter = new FileWriter(file, false);

		filewriter.write("RequestID" + "\t"
				+ "OriginPort" + "\t"
				+ "DestinationPort" + "\t"
				+ "EarliestDepartureTime" + "\t"
				+ "LatestArrivalTime" + "\t"
				+ "Demand" + "\t"
				+ "PenaltyCost" + "\n");
		// write request demands
		for(int i=0;i<in.getRequestSet().size();i++)
		{
			filewriter.write(in.getRequestSet().get(i).getRequestID() + "\t"
					+ in.getRequestSet().get(i).getOriginPort() + "\t"
					+ in.getRequestSet().get(i).getDestinationPort() + "\t"
					+ in.getRequestSet().get(i).getW_i_Earliest() + "\t"
					+ in.getRequestSet().get(i).getLatestDestinationTime() + "\t"
					+ p.getDemand()[i] + "\t"
					+ p.getPenaltyCostForDemand()[i] + "\n");
		}
		filewriter.flush();
		filewriter.close();
	}
	private void SetRequests() throws IOException {
		// set request demands
		int [] demandRequest =new int [in.getRequestSet().size()];
		String []  originOfDemand =new String  [in.getRequestSet().size()];
		String []  destinationOfDemand =new String  [in.getRequestSet().size()];
		double [] demand =new double[in.getRequestSet().size()];
		double [] demandMaximum =new double[in.getRequestSet().size()];
		double [] penaltyCostForDemand= new double[in.getRequestSet().size()];
		int x=0;
		for(Request rr: in.getRequestSet())
		{
			demandRequest[x]=rr.getRequestID();
			originOfDemand[x]=rr.getOriginPort();
			destinationOfDemand[x]=rr.getDestinationPort();
			int groupO=rr.getOriginGroup();
			int groupD=rr.getDestinationGroup();

			ODRange groupRangeRange = in.getGroupRange(groupO, groupD);

			demand[x]=groupRangeRange.getDemandLowerBound()
					+(int)(
							(groupRangeRange.getDemandUpperBound()
									-groupRangeRange.getDemandLowerBound())
									*getRandDouble()
							);
			penaltyCostForDemand [x]=groupRangeRange.getFreightLowerBound()
					+(int)(
					(groupRangeRange.getFreightUpperBound()
							- groupRangeRange.getFreightLowerBound())
									*getRandDouble());
			penaltyCostForDemand [x] = penaltyCostForDemand [x] * penaltyCoefficient;
			// variable demand = 0.05 * normal demand
			demandMaximum[x]=demand[x]*uncertainDegree;

			x=x+1;
		}

		p.setDemandRequestSet(demandRequest);
		p.setOriginOfDemand(originOfDemand);
		p.setDestinationOfDemand(destinationOfDemand);
		p.setDemand(demand);
		p.setMaximumDemandVariation(demandMaximum);
		p.setPenaltyCostForDemand(penaltyCostForDemand);

		WriteRequests();
	}
	private void SetShipRoutes(){
		// set ship route IDs set
		int [] vesselRoute =new int [in.getShipRouteSet().size()];
		int [] roundTrips = new int[in.getShipRouteSet().size()];
		int x=0;
		for(ShipRoute ss:in.getShipRouteSet())
		{
			vesselRoute[x]=ss.getShippingRouteID();
			roundTrips[x] = ss.getNumRoundTrips();
			x=x+1;
		}
		p.setShippingRouteSet(vesselRoute);
		p.setNumOfRoundTrips(roundTrips);
	}
	private void SetVessels(){
		// set vessel type
		// v[x][r] == 1: vessel x is for ship route r
		// v[x][r] == 0: otherwise
		int vessel [] =new int [in.getVesselSet().size()];
		int vesselCapacity []=new int [in.getVesselSet().size()];
		double [] vesselOperationCost =new double [in.getVesselSet().size()];
		int [][] vesselTypeAndShippingRoute =new int[in.getVesselSet().size()][in.getShipRouteSet().size()];
		int [] shippingRouteVesselNum = new int[p.getShippingRouteSet().length];
		Arrays.fill(shippingRouteVesselNum, 0);
		int x=0;
		for(Vessel v : in.getVesselSet())
		{
			vesselTypeAndShippingRoute[x][v.getRoute()-1]=1;
			shippingRouteVesselNum[v.getRoute()-1] += 1;
			vessel[x]=v.getId();
			vesselCapacity[x]=v.getCapacity();
			vesselOperationCost[x]=v.getCost();
			x=x+1;
		}
		p.setVesselTypeAndShipRoute(vesselTypeAndShippingRoute);
		p.setVesselSet(vessel);
		p.setVesselCapacity(vesselCapacity);
		p.setVesselOperationCost(vesselOperationCost);
		p.setShippingRouteVesselNum(shippingRouteVesselNum);
	}
	private void SetVesselPaths(){
		// set shipRoute And vessel ContainerPath :
		// 			if vesselPath w is shipping for shipRoute r , then shipRouteAndVesselPath[r][w] == 1
		// set arcAndVesselPath :
		// 			if travel arc nn is in vessel ContainerPath w, then arcAndVesselPath[nn][w] == 1
		int [][] shipRouteAndVesselPath = new int [in.getShipRouteSet().size()] [in.getVesselPathSet().size()];
		int[] vesselPathSet =new int [in.getVesselPathSet().size()];
		int [][] arcAndVesselPath =new int [in.getTravelingArcSet().size()][in.getVesselPathSet().size()];
		int[] VesselPathShipRouteSet =new int [in.getVesselPathSet().size()];

		for(int w = 0; w < in.getVesselPathSet().size(); w++)
		{
			int ww = in.getVesselPathSet().get(w).getVesselPathID();

			int r = in.getVesselPathSet().get(w).getRouteID() - 1;
			shipRouteAndVesselPath[r][w] = 1;
			VesselPathShipRouteSet[w] = r;

			for (int nn = 0; nn < in.getTravelingArcSet().size(); nn++)
			{
				for (int j = 0; j < in.getVesselPathSet().get(w).getPathArcIDs().length; j++)
				{
					if (in.getTravelingArcSet().get(nn).getTravelingArc_ID() == in.getVesselPathSet().get(w).getPathArcIDs()[j])
					{
						arcAndVesselPath[nn][w] = 1;
					}
				}
			}
			vesselPathSet[w] = ww;
		}

		// [nn][w]
		p.setArcAndVesselPath(arcAndVesselPath);
		// [r][w]
		p.setShipRouteAndVesselPath(shipRouteAndVesselPath);
		p.setVesselPathSet(vesselPathSet);
		p.setVesselPathShipRouteIndex(VesselPathShipRouteSet);
	}
	private void SetContainerPaths(){
		// calculate total demurrage for each path
		// demurrage = sum{transshipTime * unit demurrage}
		// arcAndPath : arcs X paths
		// arcAndPath[arc][path] == 1 : the travel arc is in path arcs
		double [] PathLoadAndDischargeCost=new double [in.getContainerPathSet().size()];
		double [] ladenPathDemurrageCost=new double [in.getContainerPathSet().size()];
		double [] emptyPathDemurrageCost=new double [in.getContainerPathSet().size()];
		double [] ladenPathCost=new double [in.getContainerPathSet().size()];
		double [] emptyPathCost=new double [in.getContainerPathSet().size()];
		int [] travelTimeOnLadenPath=new int [in.getContainerPathSet().size()];
		int [] PathSet =new int [in.getContainerPathSet().size()];
		int [][] arcAndPath  =new int [in.getTravelingArcSet().size()][in.getContainerPathSet().size()];
		int x=0;
		for(ContainerPath pp :in.getContainerPathSet())
		{
			for (int i = 0; i < in.getPortSet().size(); i++) {
				if (in.getPortSet().get(i).getPort().equals(pp.getOriginPort())){
					PathLoadAndDischargeCost[x] += in.getPortSet().get(i).getLoadingCost();
				} else if (in.getPortSet().get(i).getPort().equals(pp.getDestinationPort())) {
					PathLoadAndDischargeCost[x] += in.getPortSet().get(i).getDischargeCost();
				}
				else {
					if(pp.getTransshipment_port() != null && pp.getTransshipment_port().length > 0){
						for (int j = 0; j < pp.getTransshipment_port().length; j++) {
							if(in.getPortSet().get(i).getPort().equals(pp.getTransshipment_port()[j])){
								PathLoadAndDischargeCost[x] += in.getPortSet().get(i).getTransshipmentCost();
							}
						}
					}
				}
			}
			pp.setPathCost(PathLoadAndDischargeCost[x]);

			// why - 7 ? a setting
//			ladenPathDemurrageCost[x]=Math.max(0, 175*(pp.getTotalTransshipment_Time()-7));
//			emptyPathDemurrageCost[x]=Math.max(0, 100*(pp.getTotalTransshipment_Time()-7));

			ladenPathDemurrageCost[x]=Math.max(0, 175*pp.getTotalDemurrageTime());
			emptyPathDemurrageCost[x]=Math.max(0, 100*pp.getTotalDemurrageTime());

			ladenPathCost[x]= ladenPathDemurrageCost[x] + PathLoadAndDischargeCost[x];
			emptyPathCost[x]= emptyPathDemurrageCost[x] + PathLoadAndDischargeCost[x] * 0.5;

			travelTimeOnLadenPath[x]=pp.getPathTime();

			PathSet[x]=pp.getContainerPathID();

			for (int i = 0; i < in.getTravelingArcSet().size(); i++) {
				arcAndPath[i][x] = 0;
				for (int j = 0; j < pp.getArcsID().length; j++) {
					if(in.getTravelingArcSet().get(i).getTravelingArc_ID() == pp.getArcsID()[j])
					{
						arcAndPath[i][x] = 1;
					}
				}
			}
			++x;
		}

		p.setLadenPathDemurrageCost(ladenPathDemurrageCost);
		p.setEmptyPathDemurrageCost(emptyPathDemurrageCost);
		p.setLadenPathCost(ladenPathCost);
		p.setEmptyPathCost(emptyPathCost);
		p.setTravelTimeOnPath(travelTimeOnLadenPath);
		p.setPathSet(PathSet);
		p.setArcAndPath(arcAndPath);
	}
	private void SetArcCapacity(){
		for (int nn = 0; nn < p.getTravelingArcsSet().length; nn++) {
			double capacity = 0;
			for(int r = 0; r<p.getShippingRouteSet().length; r++)
			{
				// ω∈Ω
				for(int w=0;w<p.getVesselPathSet().length;w++)
				{
					// h∈H
					for(int h=0;h<p.getVesselSet().length;h++)
					{
						capacity +=p.getArcAndVesselPath()[nn][w]
								*p.getVesselCapacity()[h]
								*p.getShipRouteAndVesselPath()[r][w]
								*p.getVesselTypeAndShipRoute()[h][r];
					}
				}
			}

			if ( DebugEnable && GenerateParamEnable)
			{
				System.out.println("RouteID = "+in.getTravelingArcSet().get(nn).getRoute() +'\t'
						+"TravelArcID = " + in.getTravelingArcSet().get(nn).getTravelingArc_ID() +'\t'
						+"(" + in.getTravelingArcSet().get(nn).getOriginPort().toString()
						+"," + in.getTravelingArcSet().get(nn).getDestinationPort().toString() +")" +'\t'
						+"("+in.getTravelingArcSet().get(nn).getOriginTime()
						+"--"+in.getTravelingArcSet().get(nn).getDestinationTime()+")"+'\t'
						+"Total Capacity = " + capacity
				);
			}
		}
	}
	private void SetInitialEmptyContainers(){
		// calculate initial number of empty containers for each port at time 0
		// initial number of empty container in pp = total demands which origins in port pp * [0.8, 1.0]
		int[] initialEmptyContainer =new int [in.getPortSet().size()];
		int x=0;
		double alpha=0.8+0.2*getRandDouble();
		for(Port pp:in.getPortSet())
		{
			for(int i = 0; i<in.getRequestSet().size(); i++)
			{
				if(pp.getPort().equals(in.getRequestSet().get(i).getOriginPort())
						&&in.getRequestSet().get(i).getW_i_Earliest()<initialEmptyContainers)
				{
					initialEmptyContainer [x]=(int) (initialEmptyContainer [x]+alpha*p.getDemand()[i]);
				}
			}
			x=x+1;
		}
		p.setInitialEmptyContainer(initialEmptyContainer);
	}

	private void WriteRandomSampleSceneSet() throws IOException {
		String samplefilename = "R"+ in.getShipRouteSet().size() + "-T"
				+ p.getTimeHorizon() + "-Tau"+ p.getTau() + "-S" + randomSeed + "-SampleTestSet"+ ".txt";
		File file = new File(RootPath + DataPath + CasePath + samplefilename);
		if (!file.exists()) {
			try {
				file.createNewFile();
			} catch (IOException e) {
				e.printStackTrace();
			}
		}

		FileWriter filewriter = new FileWriter(file, false);
		filewriter.flush();

		double[][] sampleScenes =p.getSampleScenes();
		for(int i=0;i<p.getSampleScenes().length;i++)
		{
			filewriter.write(i + "\t");
			for(int j=0;j<p.getSampleScenes()[i].length;j++)
			{
				if(sampleScenes[i][j] != 0)
					filewriter.write(j + ",");
			}
			filewriter.write("\n");
			filewriter.flush();
		}
		filewriter.close();
	}

	private void GenerateRandomSampleSceneSet() throws IOException {
		// generate random sample scenarios
		double[][] sampleScenes = new double[numSampleScenes][in.getRequestSet().size()];
		for(int i=0;i<numSampleScenes;i++)
		{
			Set<Integer> set = new HashSet<>();
			while (set.size() < p.getTau()) {
				set.add(random.nextInt(in.getRequestSet().size()));
			}

			for(int j : set){
				sampleScenes[i][j] = 1;
			}
		}
		p.setSampleScenes(sampleScenes);

		if(WhetherWriteSampleTests)
		{
			WriteRandomSampleSceneSet();
		}

	}
}
