let scene, controls, camera, ambient, point, cloudLoader, lineLoader, renderer, container, material, loader = null; 
var gripper_ip = location.hostname;
var xmlrpc_port = '8100'; 
var parameters, timerId;
var loading = document.getElementById('waiting');
var feature_container_list = ['none'];
var tcp_pose = null;
var alertFlag = false;

//Options to be added to dat.gui
parameters = 
{
    //Point size
    changePointsize: 0.014,

    //Dims
    inner_dims: false,
    innerX : 0.01,
    innerY : 0.01, 
    innerZ : 0.01,
    outerX : 0.01,
    outerY : 0.01,
    outerZ : 0.01,

    //View
    view_method : 'fixed',
    container : 'none',
    remove_container : false,
    remove_plane : false,
    remove_plane_tol : 0.005,
    disparity_shift : 100,
    view_distance : 0.15,

    //Downsample
    leaf_size : 0.003,
    
    //Segmentation
    segmentation_method : 'dbscan',
    nbscan : 0,
    dbscan : 0,
    search_radius : 0.01,
    min_samples : 10,
    curve_threshold : 0.01,
    angle_threshold : 0.01,
    k : 20,
    n : 20,
    i_display : 0,
    j_display : 0,
    c_display : 0,

    //Filter
    x_axis_min : 0.01,
    x_axis_max : 0.01,
    y_axis_min : 0.01,
    y_axis_max : 0.01,
    z_axis_min : 0.01,
    z_axis_max : 0.01,
    cluster_size_min : 100,
    cluster_size_max : 10000000,
    filter_tol : 0.01,
    max_clouds_returned : 100,
    
    //Gripper Controls
    open_gripper: function() 
    {
        var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "open_gripper");
        var response = request.send();
        console.log(response.parseXML());
    },

    close_gripper: function() 
    {
        var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "close_gripper");
        var response = request.send();
        console.log(response.parseXML());
    },

    align_gripper_with_axis: function() 
    {
        var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "align_gripper_with_axis");
        var response = request.send();
        console.log(response.parseXML());
    },
    
    SetStartpoint: function() 
    {
        var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "get_tcp_pose_vec");
        var response = request.send();
        tcp_pose = response.parseXML();
        console.log(tcp_pose);
    },

    Scan: function() 
    {
        if(tcp_pose == null)
        {
            alert('You must set a startpoint!');
            return;
        }
        if(parameters['view_method'] != 'fixed' & !alertFlag)
        {
            var check = confirm('Scan will move robot.');
            alertFlag = true;
            if(!check) return;
        }
        
//         loading.style.display = 'flex'; 
        
//         timerId = setTimeout(function step_auto_tune_nbscan() {


        //Segment based on Parameters
        var py_lib = dat_gui_obj_to_py_dict(gui.getSaveObject());

        //get_feature_cloud(feature, view_method = 'fixed', search_location = None, view_tol=0.2, output=gui)
        var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "get_feature_capture");
        console.log(py_lib[gui.preset]);        //feature
        console.log(parameters['view_method']); //view method
        console.log(tcp_pose);                  //pose

        request.addParam(py_lib[gui.preset]);
        request.addParam(parameters['view_method']);
        if(parameters['container'] == 'none' || parameters['container'] == 'single' || parameters['container'] == 'fixed'){request.addParam(tcp_pose);}
        else
        {
            py_lib[parameters['container']]['pose'] = tcp_pose;
            request.addParam(py_lib[parameters['container']]);
        }
        request.addParam(0.2);
        request.addParam('gui');
        var response = request.send();
        console.log(response.parseXML());


        // Remove Plane
        if(parameters['remove_plane'])
        {
            var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "wizard_remove_planar_surface");
            request.addParam(parameters['remove_plane_tol']);
            var response = request.send();
            console.log(response.parseXML()); 
        }

        // Remove Container
        if(parameters['remove_container'])
        {
            var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "wizard_remove_container");
            request.addParam(py_lib[parameters['container']]);
            var response = request.send();
            console.log(response.parseXML());
        }

        // Update Scene
//         loading.style.display = 'none';
        update_scene();

        // Move back to start
        var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "movej");
        request.addParam(tcp_pose);
        var response = request.send();
        console.log(response.parseXML());
//         },100);
//         clearTimeout(timerId);
    },
    
    Segment: function() 
    { 
//         loading.style.display = 'flex';
        
//         timerId = setTimeout(function step_auto_tune_nbscan(){
        var py_lib = dat_gui_obj_to_py_dict(gui.getSaveObject());
        var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "wizard_segment_cloud");
        request.addParam(py_lib[gui.preset]);
        if(parameters['remove_plane']){request.addParam('cloud_pr');}
        else if(parameters['remove_container']){request.addParam('cloud_cr');}
        else{request.addParam('cloud_raw');}
        
        var response = request.send();
        console.log(response.parseXML());
        

//         // Update Scene
//             loading.style.display = 'none';
       update_scene();
//         }, 100);
//         clearTimeout(timerId);
    },
    
    AutoFill_Filter: function() 
    {
        console.log(gui.__controllers);
        parameters['x_axis_min'] = parameters['outerX'] - parameters['filter_tol'];
        parameters['x_axis_max'] = parameters['outerX'] + parameters['filter_tol'];
        parameters['y_axis_min'] = parameters['outerY'] - parameters['filter_tol'];
        parameters['y_axis_max'] = parameters['outerY'] + parameters['filter_tol'];
        parameters['z_axis_min'] = parameters['outerZ'] - parameters['filter_tol'];
        parameters['z_axis_max'] = parameters['outerZ'] + parameters['filter_tol'];
        
        for (var i in filter.__controllers) 
        {
            filter.__controllers[i].updateDisplay();
            console.log('gui update');
        }
        

    },

    Filter: function() 
    {
//         loading.style.display = 'flex';
//         timerId = setTimeout(function step_auto_tune_nbscan(){
        var py_lib = dat_gui_obj_to_py_dict(gui.getSaveObject());
        var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "wizard_filter_clouds_size");
        request.addParam(py_lib[gui.preset]);
        var response = request.send();
        console.log(response.parseXML());
        

        // Update Scene
//         loading.style.display = 'none';
        update_scene();
        
        // Move back to start
        var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "movej");
        request.addParam(tcp_pose);
        var response = request.send();
        console.log(response.parseXML());
  
//         },100);
//         clearTimeout(timerId);
    },
    
    Start_auto_tune_nbscan: function() 
    {
        scan.remove(auto_tune_button);
        scan.remove(n_display);
        parameters['i_display'] = 0
        parameters['j_display'] = 0
        parameters['c_display'] = 0
        auto_tune_button = scan.add(parameters,'Stop_auto_tune_nbscan').name('Stop Tunning');
        i_display = scan.add(parameters,'i_display').name('Particle');
        j_display = scan.add(parameters,'j_display').name('Iteration');
        c_display = scan.add(parameters,'c_display').name('Clouds Found');
        var n = parameters['n'];
        var py_lib = dat_gui_obj_to_py_dict(gui.getSaveObject());
        var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "init_auto_tune_nbscan");
        request.addParam(py_lib[gui.preset]);
        console.log(py_lib[gui.preset]);
        request.addParam(n);
        var response = request.send();
        console.log(response.parseXML());
        var i = 0;
        var j = 0;
        var clouds_found = 0
        var time_out = 100
        timerId = setTimeout(function step_auto_tune_nbscan() {
            // Update loop count
            parameters['j_display'] = j
            parameters['i_display'] = i
            for (var k in scan.__controllers) 
            {
                scan.__controllers[k].updateDisplay();
            }
            
            var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "step_auto_tune_nbscan");
            request.addParam(i);
            var response = request.send()
            console.log(response.parseXML())
            clouds_found = response.parseXML()
            if(clouds_found != false)
            {
                parameters['c_display'] = parseInt(clouds_found)
                for (var k in scan.__controllers) 
                {
                    scan.__controllers[k].updateDisplay();
                }
                // Update Scene
                loading.style.display = 'none';
                update_scene();
                time_out = 3000
            }
            else{ time_out = 100}
            i = i+1
            if(i >= n)
            {
                i = 0
                j = j +1
            }
            timerId = setTimeout(step_auto_tune_nbscan,time_out)
            
        },time_out); 
    },
    
    Stop_auto_tune_nbscan: function() 
    {
        scan.remove(auto_tune_button);
        scan.remove(i_display);
        scan.remove(j_display);
        scan.remove(c_display);
        n_display = scan.add(parameters, 'n').name('n');
        auto_tune_button = scan.add(parameters,'Start_auto_tune_nbscan').name('Start Tunning');
        scan.add(parameters,'Apply_nbscan_parameters').name('Apply');
        clearTimeout(timerId);
    },
    
    Apply_nbscan_parameters: function() 
    {
        parameters['segmentation_method'] = 'nbscan'
        for (var k in segmentation.__controllers) 
        {
            segmentation.__controllers[k].updateDisplay();
        }
        
        if(seg_element !=null)
        {
            segmentation.remove(seg_element);
            seg_element = null;
        }
        segmentation.removeFolder('dbscan');
        segmentation.removeFolder('nbscan');
        segmentation.removeFolder('auto nbscan');

        scan = segmentation.addFolder('nbscan');
        scan.open();
        
        
        var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "get_auto_tune_nbscan_best");
        var response = request.send()
        console.log(response.parseXML()[2])
        parameters['curve_threshold'] = parseFloat(response.parseXML()[0])
        parameters['angle_threshold'] = parseFloat(response.parseXML()[1])
        parameters['k'] = parseInt(response.parseXML()[2])
        
        scan.add(parameters, 'curve_threshold').name('Curve Threshold');
        scan.add(parameters, 'angle_threshold').name('Angle Threshold');
        scan.add(parameters, 'k').name('K');
        seg_element = segmentation.add(parameters, 'Segment');

    },
    
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ThreeJS Code
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Create a scene which will hold all our meshes to be rendered
scene = new THREE.Scene();


// var frustumSize = .5;
// var aspect = window.innerWidth / window.innerHeight;
// camera = new THREE.OrthographicCamera( frustumSize * aspect / - 2, frustumSize * aspect / 2, frustumSize / 2, frustumSize / - 2, 0.0001, 1000000 );
// // // // Create and position a camera
// //     camera = new THREE.OrthographicCamera( 
// //         window.innerWidth / - 2, 
// //         window.innerWidth / 2, 
// //         window.innerHeight / 2, 
// //         window.innerHeight / - 2, 1, 
// //         1000);

camera = new THREE.PerspectiveCamera(10, // Field of view
                                     window.innerWidth / window.innerHeight, // Aspect ratio
                                     0.1, // Near clipping pane
                                     1000); // Far clipping pane
                                    
// Reposition the camera
camera.position.set(0, 0, 10);
// Point the camera at a given coordinate
camera.lookAt(new THREE.Vector3(0, 1, 0));

// Create a renderer
renderer = new THREE.WebGLRenderer({canvas: artifactCanvas });

// // Set size
renderer.setPixelRatio(window.devicePixelRatio);
renderer.setSize(window.innerWidth, window.innerHeight);
// Set color
renderer.setClearColor(0x000000);
renderer.gammaOutput = true;
// Enable shadow mapping
renderer.shadowMap.enabled = true;
renderer.shadowMap.type = THREE.PCFSoftShadowMap;


// // Add orbit control
// controls = new THREE.OrbitControls(camera, renderer.domElement); 
// controls.target.set(0, -0.1 ,-0.1);
// controls.minDistance = 0;
// controls.maxDistance = 10;
// controls.update();
// controls.enabled = true;


controls = new THREE.TrackballControls( camera, renderer.domElement );
controls.rotateSpeed = 5;
controls.zoomSpeed = 5;
controls.panSpeed = 0.25;
controls.noZoom = false;
controls.noPan = false;
controls.staticMoving = true;
controls.dynamicDampingFactor = 0.3;
controls.keys = [ 65, 83, 68 ];
// controls.update();
// controls.enabled = true;
controls.addEventListener( 'change', render);

// Add an ambient lights
ambient = new THREE.AmbientLight(0xffffff, 0.2);
scene.add(ambient);

// Add a point light that will cast shadows
point = new THREE.PointLight(0xffffff, 1);
point.position.set(25, 50, 25);
point.castShadow = true;
point.shadow.mapSize.width = 1024;
point.shadow.mapSize.height = 1024;
scene.add(point);
//Material type (pointcloud!)
material = new THREE.PointsMaterial({ 
    size : parameters.changePointsize,
    vertexColors: THREE.FaceColors,
    transparent: false,
    opacity: 0.9,                                 
});

// Add a model
loader = new THREE.PLYLoader();

// Update Scene
update_scene();


// Append to the document
container = document.createElement('div');
document.body.appendChild(container);
document.body.appendChild(renderer.domElement);
// Add resize listener
window.addEventListener('resize', onWindowResize, false);

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Dat-Gui Code
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Remove existing entries if other is selected
dat.GUI.prototype.removeFolder = function(name) 
{
    var folder = this.__folders[name];
    if (!folder) {return;}

    folder.close();
    this.__ul.removeChild(folder.domElement.parentNode);
    delete this.__folders[name];
    this.onResize();
} 

//Create the GUI
try{
var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "load_feature_lib");
var response = request.send();
if(response == null) document.getElementById("no-server").display = flex;
var temp_obj = response.parseXML();
}

catch(err){
    alert("Gripper Server not running! (run rm_wizard_test.ipynb)");
}
// console.log(dat_gui_obj_to_py_dict(JSON.parse(temp_obj)));
console.log(py_dict_to_dat_gui_obj(temp_obj));

var gui = new dat.GUI({load: py_dict_to_dat_gui_obj(temp_obj), preset: 'default'});
//var gui = new dat.GUI({load: py_dict_to_dat_gui_obj(temp_obj), preset: 'Default'});
// var gui = new dat.GUI({load: JSON.parse(temp_obj), preset: 'Default'});
gui.remember(parameters);
gui.open();

update_feature_container_list();

//Motion Controls
gui.add(parameters, 'open_gripper').name('Open Gipper');
gui.add(parameters, 'close_gripper').name('Close Gipper');
gui.add(parameters, 'align_gripper_with_axis').name('Align Gipper');

//Change point size
gui.add(parameters,'changePointsize',0.0001,0.014).name('Change Point Size').onChange(function(){
    material.size = parameters.changePointsize;
});

// Object Properties
featureDimensions = gui.addFolder('Feature Dimensions');
outerDims = featureDimensions.addFolder('Outer Dimensions');
outerDims.add(parameters,'outerX').min(0).name('X').min(0);
outerDims.add(parameters, 'outerY').min(0).name('Y').min(0);
outerDims.add(parameters, 'outerZ').min(0).name('Z').min(0);
outerDims.open();

var innerDimX = null;
var innerDimY = null;
var innerDimZ = null;
featureDimensions.add(parameters,'inner_dims').name('Inner Dimensions').onChange(function(){
    featureDimensions.removeFolder('Inner Dimensions');
    if(parameters['inner_dims'])
    {
        innerDims = featureDimensions.addFolder('Inner Dimensions');
        innerDimX = innerDims.add(parameters,'innerX').min(0).name('X').min(0);
        innerDimY = innerDims.add(parameters, 'innerY').min(0).name('Y').min(0);
        innerDimZ = innerDims.add(parameters, 'innerZ').min(0).name('Z').min(0);
        innerDims.open();
    }
});
featureDimensions.open();

//View
view = gui.addFolder('View');
view.add(parameters, 'SetStartpoint').name('Set Startpoint');
view.add(parameters,'view_distance').name('View Distance').min(0);
view.add(parameters, 'disparity_shift').name('Disparity Shift').min(0);
view.add(parameters, 'view_method', ['fixed', 'single','cartesian','cylindrical']).name('View Method');
view.add(parameters, 'container', feature_container_list).name('Container'); //Make it so when new bin preset is created, it adds to this dropdown as well
view.add(parameters, 'Scan').name('Scan');
view.add(parameters,'remove_container').name('Remove Container').onChange(function(){
    // Remove Container
    if(parameters['remove_container'])
    {
        parameters['remove_plane'] = false
        for (var i in view.__controllers) 
        {
            view.__controllers[i].updateDisplay();
        }
        if(parameters['container'] != 'none')
        {
            var py_lib = dat_gui_obj_to_py_dict(gui.getSaveObject());
            py_lib[parameters['container']]['pose'] = tcp_pose;
            var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "wizard_remove_container");
            request.addParam(py_lib[parameters['container']]);
            var response = request.send();
            console.log(response.parseXML());
            update_scene();
        }
        else
        {
            var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "wizard_set_view_cloud");
            request.addParam('cloud_raw');
            var response = request.send();
            console.log(response.parseXML());  
            update_scene();
        }
    }
    else
    {
        var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "wizard_set_view_cloud");
        request.addParam('cloud_raw');
        var response = request.send();
        console.log(response.parseXML());  
        update_scene();
    }
});
view.add(parameters,'remove_plane').name('Remove Plane').onChange(function(){
    // Remove Plane
    if(parameters['remove_plane'])
    {
        parameters['remove_container'] = false
        for (var i in view.__controllers) 
        {
            view.__controllers[i].updateDisplay();
        }
        var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "wizard_remove_planar_surface");
        request.addParam(parameters['remove_plane_tol']);
        var response = request.send();
        console.log(response.parseXML()); 
        update_scene();
    }
    else
    {
        var request = new XmlRpcRequest("http://"+gripper_ip+":"+xmlrpc_port+"/RPC2", "wizard_set_view_cloud");
        request.addParam('cloud_raw');
        var response = request.send();
        console.log(response.parseXML());  
        update_scene();
    }
});

view.add(parameters, 'remove_plane_tol').name('Remove Plane Tolerance (mm)').min(0);

view.open();

//Downsample
downsample = gui.addFolder('Downsample Cloud');
downsample.add(parameters, 'leaf_size').name('Leaf Size (mm)  ').min(0);
downsample.open();


//Segmentation
var seg_element = null
var auto_tune_button = null
segmentation = gui.addFolder('Segmentation');
segmentation.add(parameters, 'segmentation_method', ['dbscan','nbscan','auto nbscan']).name('Segmentation Method').onChange(function(){
    if(seg_element !=null)
    {
        segmentation.remove(seg_element);
        seg_element = null;
    }
    segmentation.removeFolder('dbscan');
    segmentation.removeFolder('nbscan');
    segmentation.removeFolder('auto nbscan');
    //if user selects dbscan
    if(parameters.segmentation_method === 'dbscan')
    {
        scan = segmentation.addFolder('dbscan');
        scan.open();
        scan.add(parameters,'search_radius').min(0).name('Search Radius');
        scan.add(parameters,'min_samples').min(0).name('Min. Samples');
        seg_element = segmentation.add(parameters, 'Segment');
    }
    //if user selects nbscan
    else if(parameters.segmentation_method === 'nbscan')
    {
        scan = segmentation.addFolder('nbscan');
        scan.open();
        scan.add(parameters, 'curve_threshold').name('Curve Threshold');
        scan.add(parameters, 'angle_threshold').name('Angle Threshold');
        scan.add(parameters, 'k').name('K');
        seg_element = segmentation.add(parameters, 'Segment');

    }
    //if user selects auto nbscan
    else if(parameters.segmentation_method === 'auto nbscan')
    {
        scan = segmentation.addFolder('auto nbscan');
        scan.open();
        n_display = scan.add(parameters, 'n').name('n');
        auto_tune_button = scan.add(parameters,'Start_auto_tune_nbscan').name('Start Tunning');
    }
});
dbscan = segmentation.addFolder('dbscan');
dbscan.open();
dbscan.add(parameters,'search_radius').min(0).name('Search Radius');
dbscan.add(parameters,'min_samples').min(0).name('Min. Samples');
seg_element = segmentation.add(parameters, 'Segment');
segmentation.open();

//Filter
filter = gui.addFolder('Filter');
filter.add(parameters, 'x_axis_min').name('X Axis Min');
filter.add(parameters, 'x_axis_max').name('X Axis Max');
filter.add(parameters, 'y_axis_min').name('Y Axis Min');
filter.add(parameters, 'y_axis_max').name('Y Axis Max');
filter.add(parameters, 'z_axis_min').name('Z Axis Min');
filter.add(parameters, 'z_axis_max').name('Z Axis Max');
filter.add(parameters, 'cluster_size_min').name('Cluster Size Min').min(0);
filter.add(parameters, 'cluster_size_max').name('Cluster Size Max').min(0);
filter.add(parameters, 'max_clouds_returned').name('Max Clouds Returned');
filter.add(parameters,'AutoFill_Filter').name('Auto Fill with Tolerance');
filter.add(parameters,'filter_tol').name('Filter Tolerance');
filter.add(parameters,'Filter');
filter.open();

gui.save();

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Functions
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
function animate() {
    requestAnimationFrame( animate );
    controls.update();
    render()
}

function render() {
    renderer.render( scene, camera );
}

function onWindowResize() 
{
    camera.aspect = window.innerWidth / window.innerHeight;
    camera.updateProjectionMatrix();
    renderer.setSize(window.innerWidth, window.innerHeight);
}

//GUI Object to Python Dictionary Conversion
function dat_gui_obj_to_py_dict(obj)
{
    var py_dict = {};
    obj = obj['remembered'];
    for (var value in obj) 
    {
        var py_dict_i = {}, ds = {}, seg = {},fil = {}, obj_i = obj[value]['0'];
        py_dict_i['outer_dimensions'] = [obj_i['outerX'], obj_i['outerY'], obj_i['outerZ']];
        if(obj_i['inner_dims']){
            py_dict_i['inner_dimensions'] = [obj_i['innerX'], obj_i['innerY'], obj_i['innerZ']];
        }
        py_dict_i['view_distance'] = obj_i['view_distance'];
        py_dict_i['disparity_shift'] = obj_i['disparity_shift'];
        
        ds['descriptor'] = 'downsample';
        ds['leaf_size'] = obj_i['leaf_size'];
        if(obj_i['segmentation_method'] == 'dbscan'){
            seg['descriptor'] = 'dbscan';
            seg['search_radius'] = obj_i['search_radius'];
            seg['min_samples'] = obj_i['min_samples'];
        }
        else if(obj_i['segmentation_method'] == 'nbscan'){
            seg['descriptor'] = 'nbscan';
            seg['curve_threshold'] = obj_i['curve_threshold'];
            seg['angle_threshold'] = obj_i['angle_threshold'];
            seg['k'] = obj_i['k'];
        }
        else if(obj_i['segmentation_method'] == 'auto nbscan'){
            seg['descriptor'] = 'auto nbscan';
            seg['leaf_size'] = obj_i['leaf_size'];
        }
        fil['descriptor'] = 'filter_by_size';
        fil['x_axis'] = [obj_i['x_axis_min'], obj_i['x_axis_max']];
        fil['y_axis'] = [obj_i['y_axis_min'], obj_i['y_axis_max']];
        fil['z_axis'] = [obj_i['z_axis_min'], obj_i['z_axis_max']];
        fil['cluster_size'] = [obj_i['cluster_size_min'], obj_i['cluster_size_max']];
        fil['max_clouds_returned'] = obj_i['max_clouds_returned'];
        py_dict_i['capture_process_list'] = [ds,seg,fil];
        py_dict[value] = py_dict_i;
    }
    return py_dict;
}

//Python Dictionary to GUI Object Conversion
function py_dict_to_dat_gui_obj(py_dict)
{
    var cow = {}
    cow = {
        "preset":"default",
        "closed":false,
        "remembered": {},
        
        "folders":{
            "Feature Dimensions":{
                "preset":"default",
                "closed":false,
                "folders":{
                    "Outer Dimensions":{
                        "preset":"default",
                        "closed":false,
                        "folders":{}
                    }
                }
            },
            "View":{
                "preset":"default",
                "closed":false,
                "folders":{}
            },
            "Segmentation":{
                "preset":"default",
                "closed":false,
                "folders":{}
            },
            "Filter":{
                "preset":"default",
                "closed":false,
                "folders":{}
            }
        }
    }
    for (var value in py_dict){
        cow['remembered'][value] = {
            "0" : {
                "changePointsize": 0.014,
                "container" : 'none',
                "view_method" : 'fixed',
                "view_distance" : Number(py_dict[value]['view_distance']),
                "disparity_shift" : Number(py_dict[value]['disparity_shift']),
                
                "leaf_size" : Number(py_dict[value]['capture_process_list'][0]['leaf_size']),
                
                "segmentation_method" : String(py_dict[value]['capture_process_list'][1]['descriptor']),
                
                "x_axis_min" : Number(py_dict[value]['capture_process_list'][2]['x_axis'][0]),
                "x_axis_max" : Number(py_dict[value]['capture_process_list'][2]['x_axis'][1]),
                "y_axis_min" : Number(py_dict[value]['capture_process_list'][2]['y_axis'][0]),
                "y_axis_max" : Number(py_dict[value]['capture_process_list'][2]['y_axis'][1]),
                "z_axis_min" : Number(py_dict[value]['capture_process_list'][2]['z_axis'][0]),
                "z_axis_max" : Number(py_dict[value]['capture_process_list'][2]['z_axis'][1]),
                "cluster_size_min" : Number(py_dict[value]['capture_process_list'][2]['cluster_size'][0]),
                "cluster_size_max" : Number(py_dict[value]['capture_process_list'][2]['cluster_size'][1]),
                
                "max_clouds_returned": Number(py_dict[value]['capture_process_list'][2]['max_clouds_returned']), 
            }
        }
        if(py_dict[value]['inner_dimensions']){
            cow['remembered'][value]['0']["inner_dims"] = true;
            cow['remembered'][value]['0']["innerX"] = Number(py_dict[value]['inner_dimensions'][0]);
            cow['remembered'][value]['0']["innerY"] = Number(py_dict[value]['inner_dimensions'][1]);
            cow['remembered'][value]['0']["innerZ"] = Number(py_dict[value]['inner_dimensions'][2]);
        }
        else{cow['remembered'][value]['0']["inner_dims"] = false}
        if(py_dict[value]['outer_dimensions']){
            cow['remembered'][value]['0']["outerX"] = Number(py_dict[value]['outer_dimensions'][0]);
            cow['remembered'][value]['0']["outerY"] = Number(py_dict[value]['outer_dimensions'][1]);
            cow['remembered'][value]['0']["outerZ"] = Number(py_dict[value]['outer_dimensions'][2]);
        }
        if(cow['remembered'][value]['0']['segmentation_method'] == 'dbscan'){
            cow['remembered'][value]['0']["search_radius"] = Number(py_dict[value]['capture_process_list'][1]['search_radius']);
            cow['remembered'][value]['0']["min_samples"] = Number(py_dict[value]['capture_process_list'][1]['min_samples']);
        }
        if(cow['remembered'][value]['0']['segmentation_method'] == 'nbscan'){
            cow['remembered'][value]['0']["curve_threshold"] = Number(py_dict[value]['capture_process_list'][1]['curve_threshold']);
            cow['remembered'][value]['0']["angle_threshold"] = Number(py_dict[value]['capture_process_list'][1]['angle_threshold']);
            cow['remembered'][value]['0']["k"] = Number(py_dict[value]['capture_process_list'][1]['k']);
        }

    }
    return cow;   
}

function update_feature_container_list(){
    var obj = gui.getSaveObject();
    feature_container_list = ['none'];
    obj = obj['remembered'];
    for (var value in obj) {
        var obj_i = obj[value]['0'];
        if(obj_i['inner_dims']){
            feature_container_list.push(value);
        }
    }
}

//Clear Scene
function clear_scene(){
    while(scene.children.length > 0){ 
    scene.remove(scene.children[0]); 
    }
}

//Update Scene
function update_scene(){
    clear_scene();
    // Add a model
    const manager = initLoadingManager();
    cloudLoader = new THREE.PLYLoader(manager);
    cloudLoader.load('../../models/point_cloud.ply', function(geometry) 
    {
        geometry.computeVertexNormals();
        var figure = new THREE.Points(geometry, material);
        figure.name = 'point_cloud';
        scene.add(figure);
    }, manager.onProgress, manager.onError );
    
    $.getJSON('../../models/dem_lines.json', function(json) {
        for (i = 0; i < Object.keys(json['vertices']).length; i++){
            var lineGeometry = new THREE.Geometry();
            lineGeometry.vertices.push(new THREE.Vector3(json.vertices[i][0][0],json.vertices[i][0][1], json.vertices[i][0][2]));
            lineGeometry.vertices.push(new THREE.Vector3(json.vertices[i][1][0],json.vertices[i][1][1], json.vertices[i][1][2]));
            var line = new THREE.Line(lineGeometry, new THREE.LineBasicMaterial({ color: json.hexcolor[i]}));
            scene.add( line );
        }
    });
}

//Create Loading Bar
function initLoadingManager() {
  
  const manager = new THREE.LoadingManager();
  const progressBar = document.getElementById( 'progress' );
  const loadingOverlay = document.getElementById( 'loading-overlay' );

  let percentComplete = 1;
  let frameID = null;

  const updateAmount = 0.5; // in percent of bar width, should divide 100 evenly

  const animateBar = () => {
    percentComplete += updateAmount;
      
    if ( percentComplete >= 100 ) {
      
      progressBar.style.backgroundColor = 'red'
      percentComplete = 1;

    }

    progressBar.style.width = percentComplete + '%';

    frameID = requestAnimationFrame( animateBar )

  }

  manager.onStart = () => {
    loadingOverlay.classList.remove( 'loading-overlay-hidden' );
    // prevent the timer being set again
    // if onStart is called multiple times
    if ( frameID !== null ) return;

    animateBar();

  };

  manager.onLoad = function ( ) {

    loadingOverlay.classList.add( 'loading-overlay-hidden' );

    // reset the bar in case we need to use it again
    percentComplete = 0;
    progressBar.style.width = 0;
    cancelAnimationFrame( frameID );

  };
  
  manager.onError = function ( e ) { 
    
    console.error( e ); 
    
    progressBar.style.backgroundColor = 'red';
  
  }
  
  return manager;
}

animate()


