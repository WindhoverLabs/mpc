'use strict';

var path = require( 'path' );

const CdrPlugin = require( path.join( global.CDR_INSTALL_DIR, '/commander/classes/CdrPlugin' ) ).CdrPlugin;

module.exports = class CfeCdrPlugin extends CdrPlugin {
  constructor( urlBase ) {
    super( 'mpc', path.join( __dirname, 'web', urlBase ) );
  }

  getContent() {
    var result = {
      shortDescription: 'Multicopter Position Control',
      longDescription: 'Multicopter Position Control.',
      nodes: {
        main: {
          type: CdrPlugin.ContentType.LAYOUT,
          shortDescription: 'Main',
          longDescription: 'Main MPC.',
          filePath: '/main.lyt'
        },
        cdh: {
          type: CdrPlugin.ContentType.PANEL,
          shortDescription: 'Command and Data Handling',
          longDescription: 'Command counters.',
          filePath: '/cdh.pug'
        },
        cs: {
          type: CdrPlugin.ContentType.PANEL,
          shortDescription: 'Control State',
          longDescription: 'MPC Control State',
          filePath: '/cs.pug'
        },
        vs: {
          type: CdrPlugin.ContentType.PANEL,
          shortDescription: 'Vehicle State',
          longDescription: 'MPC Vehicle State',
          filePath: '/vs.pug'
        },
        pid: {
          type: CdrPlugin.ContentType.PANEL,
          shortDescription: 'PID',
          longDescription: 'MPC PIDs Tunint',
          filePath: '/pid.pug'
        },
        tv: {
          type: CdrPlugin.ContentType.PANEL,
          shortDescription: 'Table Values',
          longDescription: 'MPC Table Values',
          filePath: '/tv.pug'
        }
      }
    };

    return result;
  }
};

//var plugin = new CfeCdrPlugin();

//let CfeCdrPlugin = new class extends CdrPlugin {
