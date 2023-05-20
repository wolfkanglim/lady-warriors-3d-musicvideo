import * as THREE from './js/three.module.js';
import { OrbitControls } from './js/OrbitControls.js';
import { FBXLoader } from './js/FBXLoader.js';
import { Lensflare, LensflareElement } from './js/Lensflare.js';
import {ParametricGeometry} from './js/ParametricGeometry.js';


// Loading Progress using loadingManager
let loading = [];
const progressBarContainer = document.querySelector('.progress-bar-container');
const progressBar = document.getElementById('progress-bar');
const loadingManager = new THREE.LoadingManager();
const textureLoader = new THREE.TextureLoader(loadingManager);
const loader = new FBXLoader(loadingManager);

 loadingManager.onStart = function(url, itemsLoaded, itemsTotal){
console.log(`Started Loading..: ${url}`);
     if ( loading[ url ] !== undefined ) {
          loading[ url ].push( {
               onLoad: onLoad,
               onProgress: onProgress,
               onError: onError
          } );
          return;
     }
}  

 loadingManager.onProgress = function(url, itemsLoaded, itemsTotal){
     console.log(`Loading...: ${url}`);
     console.log(`Loaded...: ${itemsLoaded}`);
     console.log(`Total...: ${itemsTotal}`);
     progressBar.value = (itemsLoaded / itemsTotal) * 100;
}

loadingManager.onLoad = function(url){
     //console.log('Loading Finished');
     progressBarContainer.style.display = 'none';   
}

loadingManager.onError = function(){
     console.log(`Loading Problem...: ${url}`);
} 



// CLOTH SIMULATION using a relaxed constraints solver
               
const params = {
     enableWind: true,
     togglePins: togglePins
};

let DAMPING = 0.004;
let DRAG = 1 - DAMPING;
let MASS = 0.981;
let restDistance = 32.7;

let xSegs = 10;
let ySegs = 17;

const clothFunction = plane( restDistance * xSegs, restDistance * ySegs );

const cloth = new Cloth( xSegs, ySegs );

let GRAVITY = 81 * 1.4;
let gravity = new THREE.Vector3( 0, - GRAVITY, 0 ).multiplyScalar( MASS );

let TIMESTEP = 18 / 1000;
let TIMESTEP_SQ = TIMESTEP * TIMESTEP;

let pins = [];

const windForce = new THREE.Vector3( 0, 0, 0 );
const tmpForce = new THREE.Vector3();

function plane( width, height ) {
     return function ( u, v, target ) {
          let x = ( u - 0.5 ) * width ;
          let y = ( v + 0.5 ) * height;
          let z = 0;
          target.set( x, y, z );
     };
}

function Particle( x, y, z, mass ) {
     this.position = new THREE.Vector3();
     this.previous = new THREE.Vector3();
     this.original = new THREE.Vector3();
     this.a = new THREE.Vector3( 0, 0, 0 ); // acceleration
     this.mass = mass;
     this.invMass = 1 / mass;
     this.tmp = new THREE.Vector3();
     this.tmp2 = new THREE.Vector3();

     clothFunction( x, y, this.position ); 
     clothFunction( x, y, this.previous ); 
     clothFunction( x, y, this.original );
}

// Force -> Acceleration

Particle.prototype.addForce = function ( force ) {
     this.a.add(
          this.tmp2.copy( force ).multiplyScalar( this.invMass )
     );
};

// Performs integration

Particle.prototype.integrate = function ( timeSquare ) {
     let newPos = this.tmp.subVectors( this.position, this.previous );
     newPos.multiplyScalar( DRAG ).add( this.position );
     newPos.add( this.a.multiplyScalar( timeSquare ) );

     this.tmp = this.previous;
     this.previous = this.position;
     this.position = newPos;
     this.a.set( 0, 0, 0 );
};

const diff = new THREE.Vector3();

function satisfyConstraints( p1, p2, distance ) {
     diff.subVectors( p2.position, p1.position );
     let currentDist = diff.length();
     if ( currentDist === 0 ) return; // prevents division by 0
     let correction = diff.multiplyScalar( 1 - distance / currentDist );
     let correctionHalf = correction.multiplyScalar( 0.5 );
     p1.position.add( correctionHalf );
     p2.position.sub( correctionHalf );
}

function Cloth( w, h ) {
     w = w || 10;
     h = h || 17;
     this.w = w;
     this.h = h;

     let particles = [];
     let constraints = [];
     let u, v;

     // Create particles
     for ( v = 0; v <= h; v ++ ) {
          for ( u = 0; u <= w; u ++ ) {
               particles.push(
                    new Particle( u / w, v / h, 0, MASS )
               );
          }
     }

     // Structural

     for ( v = 0; v < h; v ++ ) {
          for ( u = 0; u < w; u ++ ) {
               constraints.push( [
                    particles[ index( u, v ) ],
                    particles[ index( u, v + 1 ) ],
                    restDistance
               ] );

               constraints.push( [
                    particles[ index( u, v ) ],
                    particles[ index( u + 1, v ) ],
                    restDistance
               ] );
          }
     }

     for ( u = w, v = 0; v < h; v ++ ) {
          constraints.push( [
               particles[ index( u, v ) ],
               particles[ index( u, v + 1 ) ],
               restDistance
          ] );
     }

     for ( v = h, u = 0; u < w; u ++ ) {
          constraints.push( [
               particles[ index( u, v ) ],
               particles[ index( u + 1, v ) ],
               restDistance
          ] );
     }

     this.particles = particles;
     this.constraints = constraints;

     function index( u, v ) {
          return u + v * ( w + 1 );
     }

     this.index = index;
}

function simulate( now ) {
     let windStrength = Math.cos( now / 7000 ) * 20 + 40;
     windForce.set( Math.sin( now / 2000 ), Math.cos( now / 3000 ), Math.sin( now / 1000 ) );
     windForce.normalize();
     windForce.multiplyScalar( windStrength );

     let i, j, il, particles, particle, constraints, constraint;

     // Aerodynamics forces

     if ( params.enableWind ) {
          let indx;
          let normal = new THREE.Vector3();
          let indices = clothGeometry.index;
          let normals = clothGeometry.attributes.normal;

          particles = cloth.particles;

          for ( i = 0, il = indices.count; i < il; i += 3 ) {
               for ( j = 0; j < 3; j ++ ) {
                    indx = indices.getX( i + j );
                    normal.fromBufferAttribute( normals, indx );
                    tmpForce.copy( normal ).normalize().multiplyScalar( normal.dot( windForce ) );
                    particles[ indx ].addForce( tmpForce );
               }
          }
     }

     for ( particles = cloth.particles, i = 0, il = particles.length; i < il; i ++ ) {
          particle = particles[ i ];
          particle.addForce( gravity );
          particle.integrate( TIMESTEP_SQ );
     }

     // Start Constraints

     constraints = cloth.constraints;
     il = constraints.length;

     for ( i = 0; i < il; i ++ ) {
          constraint = constraints[ i ];
          satisfyConstraints( constraint[ 0 ], constraint[ 1 ], constraint[ 2 ] );
     }
     
     // Floor Constraints

     for ( particles = cloth.particles, i = 0, il = particles.length; i < il; i ++ ) {
          particle = particles[ i ];
          pos = particle.position;
          if ( pos.y < -250 ) {
               pos.y = -250;
          }
     }

     // Pin Constraints

     for ( i = 0, il = pins.length; i < il; i ++ ) {
          let xy = pins[ i ];
          let p = particles[ xy ];
          p.position.copy( p.original );
          p.previous.copy( p.original );
     }
     
}

/* testing cloth simulation */
let pinsFormation = [];
pins = [ 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 ];
pinsFormation.push( pins );
pins = [ 0, 10 ];
pinsFormation.push( pins );
pins = pinsFormation[ 1 ];

function togglePins() {
     pins = pinsFormation[ ~ ~ ( Math.random() * pinsFormation.length ) ];
}


// DEFINE VARIABLES for Three
let scene, camera, renderer, analyser, uniforms, audio, controls, cameraTarget, boxMesh, clock, object, objectLeft;
let videoTexture1;

let balls = [];
let speakers = [];
let flares = [];
// 
let speed = 2.5;
let height = 1;
let offset = 0.5;
// 
let z_res = "add";
let y_res = "add";
let x_res = "add";
// 
let mixer, mixer2, mixer3, mixer4, mixer5;
let ready = false;
let clothGeometry, pos;

 const startButton = document.getElementById( 'startButton' );
startButton.addEventListener( 'click', function () {
     Ammo().then( function () {
          init();
     } );
} ); 

//const textureLoader = new THREE.TextureLoader(loadingManager);

// 
function init() {
     const container = document.getElementById( 'container' );

     renderer = new THREE.WebGLRenderer( { antialias: true } );
     renderer.shadowMap.enabled = true;
     renderer.setSize( window.innerWidth, window.innerHeight );
     renderer.setClearColor( 0x222222 ); //Black background color
     renderer.setPixelRatio( window.devicePixelRatio );
     container.appendChild( renderer.domElement );
     
     let fftSize = 128;

     const overlay = document.getElementById( 'overlay' );
     overlay.remove();

     scene = new THREE.Scene();
     clock = new THREE.Clock();

     //camera = new THREE.Camera();
     camera = new THREE.PerspectiveCamera( 75, window.innerWidth / window.innerHeight, 0.1, 10000 );
     
     camera.position.set( 0, Math.floor(Math.random() * 25 + 1), Math.random() * 20 + 40 );
     cameraTarget = new THREE.Vector3( 0, 0, 0  );

     // lights
     const ambientLight = new THREE.AmbientLight( 0xeeeeee, 0.5 );
     scene.add( ambientLight );

     const directionalLight = new THREE.DirectionalLight( 0xffffff, 1 );
     directionalLight.position.set( 100, 200, 100 );
     scene.add( directionalLight );

     let d = 200;
     directionalLight.castShadow = true;
     directionalLight.shadow.camera.left = - d;
     directionalLight.shadow.camera.right = d;
     directionalLight.shadow.camera.top = d;
     directionalLight.shadow.camera.bottom = - d;

     directionalLight.shadow.camera.near = 1;
     directionalLight.shadow.camera.far = 500;

     directionalLight.shadow.mapSize.x = 2048;
     directionalLight.shadow.mapSize.y = 2048;
     

     
     const targetLeft = new THREE.Object3D();
     const targetRight = new THREE.Object3D();
     const targetFront = new THREE.Object3D();
     const targetFrontLeft = new THREE.Object3D();

     scene.add(targetLeft);
     scene.add(targetRight);
     scene.add(targetFront);
     scene.add(targetFrontLeft);

     const spotLight = new THREE.SpotLight(0xffffff, 1);
     spotLight.position.set(0, 50, 10);
     spotLight.angle = Math.PI / 20;
     spotLight.castShadow = true;
     scene.add(spotLight);

     const spotLightLeft = new THREE.SpotLight(0xffffff, 2);
     spotLightLeft.position.set(-10, 50, 10);
     scene.add(spotLightLeft);
     spotLightLeft.castShadow = true;
     spotLightLeft.angle = Math.PI / 40;

     targetLeft.position.set(-15, 2, -5);
     spotLightLeft.target = targetLeft;

     const spotLightRight = new THREE.SpotLight(0xffffff, 2);
     spotLightRight.angle = Math.PI / 40;
     spotLightRight.castShadow = true;
     spotLightRight.position.set(10, 50, 10);
     scene.add(spotLightRight);
     targetRight.position.set(15, 2, -5);
     spotLightRight.target = targetRight;

     const spotLightFront = new THREE.SpotLight(0xffffff, 2);
     spotLightFront.angle = Math.PI / 40;
     spotLightFront.castShadow = true;
     spotLightFront.position.set(30, 50, 30);
     scene.add(spotLightFront);
     targetFront.position.set(30, 2, 25);
     spotLightFront.target = targetFront;

     let spotLightFrontLeft = new THREE.SpotLight(0xffffff, 2);
     spotLightFrontLeft.angle = Math.PI / 40;
     spotLightFrontLeft.castShadow = true;
     spotLightFrontLeft.position.set(-30, 50, 30);
     scene.add(spotLightFrontLeft);
     targetFrontLeft.position.set(-30, 2, 25);
     spotLightFrontLeft.target = targetFrontLeft;

     //video texture     
     const video = document.getElementById('video1');
     
     video.play();
     video.volume = 0.1;
     video.addEventListener( 'play', function () {
          this.currentTime = 0;
     } );
     videoTexture1 = new THREE.VideoTexture(video);
     videoTexture1.minFilter = THREE.LinearFilter;
     videoTexture1.magFilter = THREE.LinearFilter;


     //skybox
     let materialArray = [];
     let texture_ft = textureLoader.load( './assets/textures/lightblue/front.png');
     let texture_bk = textureLoader.load( './assets/textures/lightblue/back.png');
     let texture_up = textureLoader.load( './assets/textures/lightblue/top.png');
     let texture_dn = textureLoader.load( './assets/textures/lightblue/bot.png');
     let texture_rt = textureLoader.load( './assets/textures/lightblue/right.png');
     let texture_lf = textureLoader.load( './assets/textures/lightblue/left.png');     
          
     materialArray.push(new THREE.MeshBasicMaterial( { map: texture_rt }));
     materialArray.push(new THREE.MeshBasicMaterial( { map: texture_lf }));
     materialArray.push(new THREE.MeshBasicMaterial( { map: texture_up }));
     materialArray.push(new THREE.MeshBasicMaterial( { map: texture_dn }));
     materialArray.push(new THREE.MeshBasicMaterial( { map: texture_ft }));
     materialArray.push(new THREE.MeshBasicMaterial( { map: texture_bk }));

     for (let i = 0; i < 6; i++){
          materialArray[i].side = THREE.BackSide;
     }

     let skyboxGeo = new THREE.BoxGeometry( 10000, 10000, 10000);
     let skybox = new THREE.Mesh( skyboxGeo, materialArray );
     scene.add( skybox );  

     // floor
     const floorMat = new THREE.MeshStandardMaterial( {
          roughness: 0.8,
          color: 0xffffff,
          metalness: 0.2,
          bumpScale: 0.005,
          side: THREE.DoubleSide,
     } );

     textureLoader.load( "./assets/textures/03_default_normal.png", function ( map ) {
          map.wrapS = THREE.RepeatWrapping;
          map.wrapT = THREE.RepeatWrapping;
          map.anisotropy = 16;
          map.repeat.set( 30, 25 );
          map.encoding = THREE.sRGBEncoding;
          floorMat.map = map;
          floorMat.needsUpdate = true;
     } );

     const floorGeometry = new THREE.PlaneGeometry( 100, 100 );
     const floorMesh = new THREE.Mesh( floorGeometry, floorMat );
     floorMesh.receiveShadow = true;
     floorMesh.rotation.x = -Math.PI * 0.5;
     scene.add( floorMesh );
     // 
     const taegukTexture = textureLoader.load('./assets/images/taeguk5.png');
     let centerFloor = new THREE.Mesh(new THREE.CylinderGeometry(9, 9, 0.002, 36, 36), new THREE.MeshPhongMaterial({
          color: 0xeeeeee,
          map: taegukTexture,
          side: THREE.DoubleSide
     }));
     centerFloor.receiveShadow = true;
     scene.add(centerFloor);


     // LOGO
     let logoMat = new THREE.MeshStandardMaterial( {
          roughness: 1,
          color: 0xffffff,
          bumpScale: 0.02,
          metalness: 0
     } );

     textureLoader.load( "./assets/images/compact-cd.png", function ( map ) {
          map.wrapS = THREE.RepeatWrapping;
          map.wrapT = THREE.RepeatWrapping;
          map.anisotropy = 30;
          map.repeat.set( 1, 1 );
          map.encoding = THREE.sRGBEncoding;
          logoMat.map = map;
          logoMat.needsUpdate = true;
     } );

     textureLoader.load("./assets/images/spark1.png", function ( map ) {
          map.wrapS = THREE.RepeatWrapping;
          map.wrapT = THREE.RepeatWrapping;
          map.anisotropy = 30;
          map.repeat.set( 1, 1 );
          logoMat.bumpMap = map;
          logoMat.needsUpdate = true;
     } );

     // Logo podium
     let logoMesh = new THREE.Object3D();
     let logoMeshLeft = new THREE.Object3D();
     let logoMeshFront = new THREE.Object3D();
     let logoMeshFrontRight = new THREE.Object3D();

     const logoGeometry = new THREE.BoxGeometry( 4, 2, 4 );				
     logoMesh = new THREE.Mesh( logoGeometry, logoMat );
     logoMesh.position.set( 15, 1, -5 );
     logoMesh.castShadow = true;
     logoMesh.receiveShadow = true;
     scene.add( logoMesh );

     logoMeshLeft = new THREE.Mesh(logoGeometry, logoMat);
     logoMeshLeft.position.set( -15, 1, -5 );
     logoMeshLeft.castShadow = true;
     logoMeshLeft.receiveShadow = true;
     scene.add( logoMeshLeft );

     logoMeshFront = new THREE.Mesh( logoGeometry, logoMat );
     logoMeshFront.position.set( 30, 1, 25 );
     logoMeshFront.castShadow = true;
     logoMeshFront.receiveShadow = true;
     scene.add( logoMeshFront );
     
     logoMeshFrontRight = new THREE.Mesh(logoGeometry, logoMat);
     logoMeshFrontRight.position.set( -30, 1, 25 );
     logoMeshFrontRight.castShadow = true;
     logoMeshFrontRight.receiveShadow = true;
     scene.add( logoMeshFrontRight );

     // LOAD AUDIO
     let listener = new THREE.AudioListener();
     audio = new THREE.Audio( listener );     
     
     //let file = '../study/assets/cancunstyle.mp3';
     let file = './assets/audios/ladywarriors.mp3';

     // PLAY AUDIO FILE
     if ( /(iPad|iPhone|iPod)/g.test( navigator.userAgent ) ) {
          const loader = new THREE.AudioLoader(loadingManager);
          loader.load( file, function ( buffer ) {
               audio.setBuffer( buffer );
               audio.setVolume(0.5);
               audio.play();               
          });
     } else {
          const mediaElement = new Audio( file );
          mediaElement.loop = true;
          mediaElement.play();
          audio.setMediaElementSource( mediaElement );          
     }

     //  AUDIO ANALYZER
     analyser = new THREE.AudioAnalyser( audio, fftSize );
     //
     uniforms = {
          tAudioData: { value: new THREE.DataTexture( analyser.data, fftSize / 2, 1, THREE.LuminanceFormat )}
     };	

     // LOAD MODEL Animation
     //const loader = new FBXLoader(loadingManager);
     
     loader.load( './assets/models/Thriller Part 4.fbx', function ( object ) {
          mixer = new THREE.AnimationMixer( object );
          let action = mixer.clipAction( object.animations[ 0 ] );
          action.play();
          object.traverse( function ( child ) {
               if ( child.isMesh ) {
                    child.castShadow = true;
                    child.receiveShadow = true;
               }
          } );

          object.scale.set( 0.027, 0.025, 0.027 );
          object.position.set( 0, 0, -5 );
          scene.add( object );
          ready = true;
     } );
     
     loader.load( './assets/models/Samba Dancing.fbx', function ( object ) {
          mixer2 = new THREE.AnimationMixer( object );
          let action = mixer2.clipAction( object.animations[ 0 ] );
          action.play();
          object.traverse( function ( child ) {
               if ( child.isMesh ) {
                    child.castShadow = true;
                    child.receiveShadow = true;
               }
          } );

          object.scale.set( 0.027, 0.025, 0.027 );
          object.position.set( 15, 2, -5 );
          scene.add( object );
          ready = true;
     } );

     loader.load( './assets/models/Tut Hip Hop Dance.fbx', function ( object ) {
          mixer3 = new THREE.AnimationMixer( object );
          let action = mixer3.clipAction( object.animations[ 0 ] );
          action.play();
          object.traverse( function ( child ) {
               if ( child.isMesh ) {
                    child.castShadow = true;
                    child.receiveShadow = true;
               }
          } );

          object.scale.set( 0.027, 0.025, 0.027 );
          object.position.set( -15, 2, -5 );
          scene.add( object );
          ready = true;
     } );

     loader.load( './assets/models/Samba Dancing.fbx', function ( object ) {
          mixer4 = new THREE.AnimationMixer( object );
          let action = mixer4.clipAction( object.animations[ 0 ] );
          action.play();
          object.traverse( function ( child ) {
               if ( child.isMesh ) {
                    child.castShadow = true;
                    child.receiveShadow = true;
               }
          } );

          object.scale.set( 0.027, 0.025, 0.027 );
          object.position.set( 30, 2, 25 );
          scene.add( object );
          ready = true;
     } );

     loader.load( './assets/models/Tut Hip Hop Dance.fbx', function ( object ) {
          mixer5 = new THREE.AnimationMixer( object );
          let action = mixer5.clipAction( object.animations[ 0 ] );
          action.play();
          object.traverse( function ( child ) {
               if ( child.isMesh ) {
                    child.castShadow = true;
                    child.receiveShadow = true;
               }
          } );

          object.scale.set( 0.027, 0.025, 0.027 );
          object.position.set( -30, 2, 25 );
          scene.add( object );
          ready = true;
     } );

     
     //ADD BALLS AND 'SPEAKERS'
     let radius = 8.5;

     let ballGeometry = new THREE.SphereGeometry( 0.25, 32, 16 );
     ballGeometry.translate( 0, 0.3, 0 );

     //  r , b, h, sides, t
     // create objects when audio buffer is loaded
     for ( let i = 0; i < uniforms.tAudioData.value.image.data.length; i ++ ) {
          let s = i / uniforms.tAudioData.value.image.data.length * Math.PI * 2;

          let speakerMaterial = new THREE.MeshStandardMaterial();
          speakerMaterial.roughness = 0.5 * Math.random();
          speakerMaterial.metalness =  0.5;
          speakerMaterial.color.setHSL( Math.random(), 1.0, 0.5 );

          let speaker = new THREE.Mesh( new THREE.CylinderGeometry( 0.25, 0.125, 0.25, 100, 2 ), speakerMaterial );
          speaker.castShadow = true;
          speaker.receiveShadow = true;
          
          speaker.position.x = radius * Math.cos( s );
          speaker.position.z = radius * Math.sin( s );
          speaker.position.y = 0.025;

          let ballMaterial = new THREE.MeshStandardMaterial();
          ballMaterial.roughness = 0.5 * Math.random() + 0.25;
          ballMaterial.metalness =  0.5;
          ballMaterial.color.setHSL( Math.random(), 1.0, 0.3 );

          let ball = new THREE.Mesh( ballGeometry, ballMaterial );
          ball.castShadow = true;
          ball.receiveShadow = true;
          ball.userData.down = false;

          ball.position.x = radius * Math.cos( s );
          ball.position.z = radius * Math.sin( s );

          scene.add( ball );
          scene.add( speaker );
          speakers.push( speaker );
          balls.push( ball );
     }     

      // BRICK WALL
     
     let cubeMat = new THREE.MeshStandardMaterial( {
          roughness: 0.9,
          color: 0xcccccc,
          bumpScale: 0.002,
          metalness: 0.1
     } );

     let wallMat = new THREE.MeshStandardMaterial({
          map: videoTexture1,
          side: THREE.FrontSide,
          toneMapped: false,
     })

     textureLoader.load( "./assets/images/water-surface.jpg", function ( map ) {
          map.wrapS = THREE.RepeatWrapping;
          map.wrapT = THREE.RepeatWrapping;
          map.anisotropy = 10;
          map.repeat.set( 1, 5 );
          map.encoding = THREE.sRGBEncoding;
          cubeMat.map = map;
          cubeMat.needsUpdate = true;
     } );

      textureLoader.load( "./assets/images/water-surface.jpg", function ( map ) {
          map.wrapS = THREE.RepeatWrapping;
          map.wrapT = THREE.RepeatWrapping;
          map.anisotropy = 10;
          map.repeat.set(1, 10 );
          cubeMat.bumpMap = map;
          cubeMat.needsUpdate = true;
     } ); 

     // Walls back
     let boxGeometry = new THREE.BoxGeometry( 40, 30, .3);
     let boxSideGeometry = new THREE.BoxGeometry(.3, 20, 30 );
     let boxMesh = new THREE.Mesh( boxGeometry, wallMat );
     boxMesh.position.set( 0, 15, -10 );
     boxMesh.castShadow = true;
     boxMesh.receiveShadow = true;
     scene.add( boxMesh );
     camera.lookAt(boxMesh.position);

     let boxMeshLeft = new THREE.Mesh( boxSideGeometry, wallMat );
     boxMeshLeft.position.set( -25, 10, -10 );
     boxMeshLeft.rotation.y = Math.PI/2;
     boxMeshLeft.castShadow = true;
     boxMeshLeft.receiveShadow = true;
     scene.add( boxMeshLeft );
     
     let boxMeshRight = new THREE.Mesh( boxSideGeometry, wallMat );
     boxMeshRight.position.set( 25, 10, -10 );
     boxMeshRight.rotation.y = Math.PI/2;
     boxMeshRight.castShadow = true;
     boxMeshRight.receiveShadow = true;

     scene.add( boxMeshRight );

     // poles right
     let poleGeo = new THREE.BoxGeometry( 1, 30, 1 );
     let poleMat = new THREE.MeshLambertMaterial();
     //pole right
     const poleMesh = new THREE.Mesh( poleGeo, poleMat );
     poleMesh.position.set(30, 15, -10);
     poleMesh.receiveShadow = true;
     poleMesh.castShadow = true;
     scene.add( poleMesh );
     
     const poleMeshR = new THREE.Mesh( poleGeo, poleMat );
     poleMeshR.position.set(30, 15, 10);
     poleMeshR.receiveShadow = true;
     poleMeshR.castShadow = true;
     scene.add( poleMeshR );

     //pole left
     const poleLeft = new THREE.Mesh(poleGeo, poleMat);
     poleLeft.position.x = -30;
     poleLeft.position.z = -10;
     poleLeft.position.y = 15;
     poleLeft.castShadow = true;
     poleLeft.receiveShadow = true;

     scene.add(poleLeft);
     const poleLeft2 = new THREE.Mesh(poleGeo, poleMat);
     poleLeft2.position.x = -30;
     poleLeft2.position.z = 10;
     poleLeft2.position.y = 15;
     poleLeft2.castShadow = true;
     poleLeft2.receiveShadow = true;
     scene.add(poleLeft2);

     const barRight = new THREE.Mesh( new THREE.BoxGeometry( 0.5, 0.5, 20 ), poleMat );
     barRight.position.y = 28;
     barRight.position.x = 30;
     barRight.position.z = 0;
     barRight.receiveShadow = true;
     barRight.castShadow = true;
     scene.add( barRight );

     const barLeft = new THREE.Mesh(new THREE.BoxGeometry(0.5, 0.5, 20), poleMat);
     barLeft.position.y = 28;
     barLeft.position.x = -30;
     barLeft.castShadow = true;
     barLeft.receiveShadow = true;
     scene.add(barLeft);

     // poles left

     const gg = new THREE.BoxGeometry( 2, 2, 2 );
     const ggMesh = new THREE.Mesh( gg, cubeMat );
     ggMesh.position.y = 0;
     ggMesh.position.x = 30;
     ggMesh.position.z = 10;
     ggMesh.receiveShadow = true;
     ggMesh.castShadow = true;
     scene.add( ggMesh );

     const ggRight = new THREE.Mesh( gg, cubeMat );
     ggRight.position.y = 0;
     ggRight.position.x = 30;
     ggRight.position.z = - 10;
     ggRight.receiveShadow = true;
     ggRight.castShadow = true;
     scene.add( ggRight );
     
     //gg left 
     const ggLeft = new THREE.Mesh( gg, cubeMat );
     ggLeft.position.set(-30, 0, -10);				
     ggLeft.receiveShadow = true;
     ggLeft.castShadow = true;
     scene.add( ggLeft );

     const ggLeft2 = new THREE.Mesh( gg, cubeMat );
     ggLeft2.position.set(-30, 0, 10);				
     ggLeft2.receiveShadow = true;
     ggLeft2.castShadow = true;
     scene.add( ggLeft2 );     

     // cloth material
  
     taegukTexture.anisotropy = 20;

     let clothMaterial = new THREE.MeshPhongMaterial( {
          map: taegukTexture,
          side: THREE.DoubleSide,
          alphaTest: 0.8
     });

     // cloth geometry
     clothGeometry = new ParametricGeometry( clothFunction, cloth.w, cloth.h );

     // cloth mesh
     object = new THREE.Mesh( clothGeometry, clothMaterial );
     object.position.set( 30, 12.5, 0 );
     object.scale.set(0.05, 0.054, 0.05);
     object.rotation.y = 1.65;
     object.castShadow = true;
     object.receiveShadow = true;
     object.frustumCulled = false;
     scene.add( object );

     object.customDepthMaterial = new THREE.MeshDepthMaterial( {
          depthPacking: THREE.RGBADepthPacking,
          map: taegukTexture,
          alphaTest: 0.95
     });
     
     //cloth left 			
    
     let clothLeftMaterial = new THREE.MeshLambertMaterial( {
          map: taegukTexture,
          side: THREE.DoubleSide,
          alphaTest: 0.5
     });
     objectLeft = new THREE.Mesh( clothGeometry, clothLeftMaterial );
     objectLeft.position.set( -30, 12.5, 0 );
     objectLeft.scale.set(0.05, 0.054, 0.05);
     objectLeft.rotation.y = 1.45;
     objectLeft.castShadow = true;
     objectLeft.receiveShadow = true;
     objectLeft.frustumCulled = false;
     scene.add( objectLeft );

     objectLeft.customDepthMaterial = new THREE.MeshDepthMaterial( {
          depthPacking: THREE.RGBADepthPacking,
          map: taegukTexture,
          alphaTest: 0.65
     });

     // Flare lights
     addLight( Math.random(), 0.9, 0.5, 10, 33, - 10 );
     addLight( Math.random(), 0.8, 0.5, - 10, 33, - 10 );
     addLight( Math.random(), 0.8, 0.5, - 31, 30, -10 );
     addLight( Math.random(), 0.8, 0.5, 30, 31, -10 );
     addLight( Math.random(), 0.8, 0.5, - 30, 31, 10 );
     addLight( Math.random(), 0.8, 0.5, 30, 31, 10 );
     
     controls = new OrbitControls( camera, renderer.domElement );			
     
     window.addEventListener( 'resize', onResize, false );

     animate( 0 );				
}

// lensflares
let textureFlare = textureLoader.load( './assets/images/spark1.png'  );

function addLight( h, s, l, x, y, z ) {
     let light = new THREE.PointLight( 0xffffff, 0.81, 20 );
     light.color.setHSL( h, s, l );
     light.position.set( x, y, z );
     scene.add( light );

     let lensflare = new Lensflare();
     lensflare.addElement( new LensflareElement( textureFlare, 500, 0, light.color ) );
     lensflare.addElement( new LensflareElement( textureFlare, 20, 0 ) );
     lensflare.addElement( new LensflareElement( textureFlare, 40, 0 ) );
     lensflare.addElement( new LensflareElement( textureFlare, 60, 0 ) );
     lensflare.addElement( new LensflareElement( textureFlare, 40, 0 ) );
     light.add( lensflare );
     flares.push(light)
}

function onResize() {
     camera.aspect = window.innerWidth / window.innerHeight;
     camera.updateProjectionMatrix();
     renderer.setSize( window.innerWidth, window.innerHeight );
}

function animate( now ) {
     requestAnimationFrame( animate );
     simulate( now );
     videoTexture1.needsUpdate = true;

     let delta = clock.getDelta();
     if ( mixer ) mixer.update( delta / 0.8 );
     if ( mixer2 ) mixer2.update( delta / 0.8 );
     if ( mixer3 ) mixer3.update( delta / 0.8 );
     if ( mixer4 ) mixer4.update( delta / 0.8 );
     if ( mixer5 ) mixer5.update( delta / 0.8 );
    
     render();
}

function render() {		               
     // Cloth
     let p = cloth.particles;
     for ( let i = 0, il = p.length; i < il; i ++ ) {
          let v = p[ i ].position;
          clothGeometry.attributes.position.setXYZ( i, v.x, v.y, v.z );
     }

     clothGeometry.attributes.position.needsUpdate = true;
     clothGeometry.computeVertexNormals();

     analyser.getFrequencyData();
     // Audio listener
     uniforms.tAudioData.value.needsUpdate = true;
     //console.log(uniforms.tAudioData.value);     

     let time = clock.getElapsedTime();

     for ( let i = 0; i < balls.length; i ++ ) {
          let ball = balls[ i ];
          let speaker = speakers[ i ];
          let previousHeight = ball.position.y;
          // Change ball position
          ball.position.y = Math.abs( Math.sin( i * offset + ( time * speed ) ) * height * (uniforms.tAudioData.value.image.data[i] * 0.03) );					
          // Scale speakers
          speaker.scale.y = Math.abs( Math.sin( i * offset + ( time * speed ) ) * (uniforms.tAudioData.value.image.data[i] * 0.05) * 0.9 );

          if ( ball.position.y < previousHeight ) {
               ball.userData.down = false;
          } else {
               if ( ball.userData.down === true ) {
                    // ball changed direction from down to up
                    ball.userData.down = false;
               }
          }
     }
     // Change flare colors and intensity
     for (let i = flares.length - 1; i >= 0; i--) {
          flares[i].color.setHSL( Math.random() * (Math.abs( Math.sin( i * offset + ( time * speed * 0.01) ) * height * (uniforms.tAudioData.value.image.data[i] * 0.03) ) ) * 0.05, Math.random(), Math.random() * 0.25 );
          flares[i].intensity =  uniforms.tAudioData.value.image.data[Math.floor(Math.random() * 30 + 1)] * 0.001;
     }

     // Animate camera
     if (camera.position.x > 15.9) {
          x_res = "sub";					
     }else if (camera.position.z < - 15.9){
          x_res = "add";
     }

     if (x_res === "add") {
          if (camera.position.x < 15.9) {
               camera.position.x = camera.position.x + 0.005;
          }else{
               camera.position.x = camera.position.x + 0.004;
          }
     }else{
          if (camera.position.x > - 15.9) {
               camera.position.x = camera.position.x - 0.01;
          }else{
               camera.position.x = camera.position.x + 0.004;
               x_res = "add";
          }
     }				

     if (camera.position.y > 25) {
          y_res = "sub";					
     }else if (camera.position.z < 0){
          y_res = "add";
     }

     if (y_res === "add") {
          if (camera.position.y < 0) {
               camera.position.y = camera.position.y + 0.01;
          }else{
               camera.position.y = camera.position.y + 0.02;
          }
          
     }else{
          if (camera.position.y < 0) {
               y_res = "add";               
          }	
          camera.position.y = camera.position.y - 0.01;				
     }

     if (camera.position.z > 19.5) {
          z_res = "sub";					
     }else if (camera.position.z < - 9.5){
          z_res = "add";
     }

     if (z_res === "add") {
          camera.position.z = camera.position.z + 0.01;
     }else{
          camera.position.z = camera.position.z - 0.01;
     }     
     
     camera.lookAt( cameraTarget );
     

     renderer.render( scene, camera );

};